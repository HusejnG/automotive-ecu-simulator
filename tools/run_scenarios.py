#!/usr/bin/env python3
"""
run_scenarios.py

Integration test harness: runs the ecu_simulator binary as a black box,
parses its --json event stream, and asserts that the system as a whole
behaved correctly.

This tests something the C++ unit tests can't: those exercise classes in
isolation with hand-fed inputs. This runs the actual built program end to
end and checks the observable result -- closer to how a real test rig
validates an ECU.

Exit code 0 = all scenarios passed, 1 = at least one failed, so this can
run in CI alongside ctest.

Usage:
    python3 run_scenarios.py [path/to/ecu_simulator]
"""

import json
import subprocess
import sys
from pathlib import Path

from signal_codec import (
    BMS_STATUS_SIGNALS,
    DIAGNOSTIC_SESSIONS,
    UDS_DIDS,
    BitReader,
    decode_frame,
    decode_signal,
    parse_hex,
)

UDS_RESPONSE_ID = 0x7A8
BMS_STATUS_ID = 0x200


def find_simulator() -> Path:
    """Looks in the usual build output locations across platforms."""
    if len(sys.argv) > 1:
        return Path(sys.argv[1])

    here = Path(__file__).resolve().parent.parent
    candidates = [
        here / "build" / "ecu_simulator",                  # Linux/macOS
        here / "build" / "Release" / "ecu_simulator.exe",  # MSVC Release
        here / "build" / "Debug" / "ecu_simulator.exe",    # MSVC Debug
        here / "build" / "ecu_simulator.exe",              # MinGW
    ]
    for path in candidates:
        if path.exists():
            return path

    print("ERROR: could not find ecu_simulator binary. Build the project "
          "first, or pass the path as an argument.", file=sys.stderr)
    sys.exit(2)


def run_simulator(binary: Path) -> list[dict]:
    """Runs the simulator in JSON mode and returns the parsed events."""
    result = subprocess.run(
        [str(binary), "--json"],
        capture_output=True,
        text=True,
        timeout=30,
    )
    if result.returncode != 0:
        print(f"ERROR: simulator exited with code {result.returncode}", file=sys.stderr)
        print(result.stderr, file=sys.stderr)
        sys.exit(2)

    events = []
    for line in result.stdout.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError:
            print(f"WARNING: skipping unparseable line: {line!r}", file=sys.stderr)
    return events


class Scenario:
    """Collects pass/fail checks under a named scenario."""

    def __init__(self, name: str):
        self.name = name
        self.failures: list[str] = []
        self.checks = 0

    def expect(self, condition: bool, description: str):
        self.checks += 1
        if not condition:
            self.failures.append(description)

    def expect_equal(self, actual, expected, description: str):
        self.checks += 1
        if actual != expected:
            self.failures.append(f"{description}: expected {expected!r}, got {actual!r}")

    def expect_near(self, actual: float, expected: float, tolerance: float, description: str):
        self.checks += 1
        if abs(actual - expected) > tolerance:
            self.failures.append(
                f"{description}: expected {expected} +/- {tolerance}, got {actual}")

    @property
    def passed(self) -> bool:
        return not self.failures

    def report(self) -> bool:
        status = "PASS" if self.passed else "FAIL"
        print(f"[{status}] {self.name} ({self.checks} checks)")
        for failure in self.failures:
            print(f"       - {failure}")
        return self.passed


# ---- scenarios --------------------------------------------------------


def check_charge_cycle(events) -> Scenario:
    s = Scenario("BMS charge cycle reaches Balancing at full charge")
    states = [e for e in events
              if e["event"] == "state" and e["scenario"] == "charge_cycle"]

    s.expect(len(states) >= 4, "expected at least 4 charge-cycle ticks")
    if len(states) < 4:
        return s

    by_label = {e["label"]: e["state"] for e in states}
    s.expect_equal(by_label.get("idle"), "Sleep", "idle state")
    s.expect_equal(by_label.get("charging"), "Charging", "charging state")
    s.expect_equal(by_label.get("full"), "Balancing",
                   "state once SoC reaches 100% while current flows")
    return s


def check_bms_status_frames(events) -> Scenario:
    s = Scenario("BMS status frames decode to the expected physical values")
    frames = [e for e in events if e["event"] == "frame" and e["id"] == BMS_STATUS_ID]

    s.expect(len(frames) >= 4, "expected at least 4 BMS status frames")
    if not frames:
        return s

    s.expect_equal(frames[0]["dlc"], 6, "BMS status frame length (4 signals = 48 bits)")

    # First frame in the demo is the idle step: 380.0 V, 0 A, 25 C, 40 %.
    decoded = decode_frame(BMS_STATUS_SIGNALS, parse_hex(frames[0]["data"]))
    s.expect_near(decoded["pack_voltage_v"], 380.0, 0.01, "idle pack voltage")
    s.expect_near(decoded["pack_current_a"], 0.0, 0.1, "idle pack current")
    s.expect_near(decoded["state_of_charge"], 40.0, 0.5, "idle state of charge")
    s.expect_near(decoded["temperature_c"], 25.0, 1.0, "idle temperature")
    return s


def check_uds_session_control(events) -> Scenario:
    s = Scenario("UDS DiagnosticSessionControl switches to Extended")
    responses = [e for e in events
                 if e["event"] == "frame" and e["id"] == UDS_RESPONSE_ID]

    session_responses = [r for r in responses
                         if parse_hex(r["data"])[0] == 0x50]  # 0x10 + 0x40
    s.expect(len(session_responses) >= 1, "expected a positive session-control response")
    if not session_responses:
        return s

    payload = parse_hex(session_responses[0]["data"])
    s.expect_equal(DIAGNOSTIC_SESSIONS.get(payload[1]), "Extended",
                   "session echoed in the positive response")
    return s


def check_uds_security_access(events) -> Scenario:
    s = Scenario("UDS SecurityAccess seed/key exchange unlocks the ECU")
    security = [e for e in events if e["event"] == "security"]

    s.expect(len(security) == 1, "expected exactly one security-status event")
    if not security:
        return s

    s.expect_equal(security[0]["unlocked"], True,
                   "security state after a correctly computed key")

    # Independently verify the key the tester sent matches the documented
    # transform, rather than trusting the simulator's own verdict.
    seed = security[0]["seed"]
    expected_key = seed ^ 0xA5A5
    key_requests = [e for e in events
                    if e["event"] == "frame" and e["id"] == 0x7A0
                    and parse_hex(e["data"])[:2] == bytes([0x27, 0x02])]
    s.expect(len(key_requests) >= 1, "expected a sendKey request frame")
    if key_requests:
        payload = parse_hex(key_requests[0]["data"])
        sent_key = (payload[2] << 8) | payload[3]
        s.expect_equal(sent_key, expected_key, "key derived from seed")
    return s


def check_uds_read_data(events) -> Scenario:
    s = Scenario("UDS ReadDataByIdentifier returns live BMS values")
    responses = [e for e in events
                 if e["event"] == "frame" and e["id"] == UDS_RESPONSE_ID
                 and parse_hex(e["data"])[0] == 0x62]  # 0x22 + 0x40

    s.expect(len(responses) >= 2, "expected at least two RDBI responses")
    if not responses:
        return s

    voltage_responses = []
    for r in responses:
        payload = parse_hex(r["data"])
        did = (payload[1] << 8) | payload[2]
        if did == 0x1001:
            voltage_responses.append(payload[3:])

    s.expect(len(voltage_responses) >= 1, "expected a response for DID 0x1001")
    if voltage_responses:
        reader = BitReader(voltage_responses[0])
        voltage = decode_signal(reader, UDS_DIDS[0x1001])
        # At that point in the demo the pack is at 400.0 V (the "full" step).
        s.expect_near(voltage, 400.0, 0.01,
                      "voltage read over UDS matches the BMS's live value")
    return s


def check_stuck_sensor_fault(events) -> Scenario:
    s = Scenario("Stuck sensor trips a fault only after the streak threshold")
    states = [e for e in events
              if e["event"] == "state" and e["scenario"] == "stuck_sensor"]

    s.expect(len(states) >= 6, "expected at least 6 stuck-sensor ticks")
    if len(states) < 6:
        return s

    # The frozen reading is individually plausible, so nothing should fault
    # early -- only the staleness itself should trigger it.
    for event in states[:5]:
        s.expect(event["state"] != "Fault",
                 f"tick {event['tick']} faulted before the threshold was reached")

    s.expect_equal(states[5]["state"], "Fault",
                   "state at the tick where the streak threshold is crossed")
    return s


def check_lossy_relay(events) -> Scenario:
    s = Scenario("Lossy relay drops exactly the expected frames")
    relay = [e for e in events if e["event"] == "relay"]

    s.expect(len(relay) == 1, "expected exactly one relay-statistics event")
    if not relay:
        return s

    stats = relay[0]
    s.expect_equal(stats["seen"], 4, "frames seen by the relay")
    s.expect_equal(stats["dropped"], 2, "frames dropped (every 2nd of 4)")
    s.expect_equal(stats["delivered"], 2, "frames delivered downstream")
    s.expect_equal(stats["seen"] - stats["dropped"], stats["delivered"],
                   "relay counters are self-consistent")
    return s


SCENARIOS = [
    check_charge_cycle,
    check_bms_status_frames,
    check_uds_session_control,
    check_uds_security_access,
    check_uds_read_data,
    check_stuck_sensor_fault,
    check_lossy_relay,
]


def main() -> int:
    binary = find_simulator()
    print(f"Running integration scenarios against: {binary}\n")

    events = run_simulator(binary)
    print(f"Captured {len(events)} events from the simulator\n")

    results = [scenario(events).report() for scenario in SCENARIOS]

    passed = sum(results)
    total = len(results)
    print(f"\n{passed}/{total} scenarios passed")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
