# Automotive ECU Simulator

A C++ simulation of an automotive electronic control unit (ECU) network:
a virtual CAN bus, a Battery Management System modeled as an AUTOSAR-style
software component, UDS diagnostic services, and a fault-injection layer
to validate fail-safe behavior. Built to demonstrate practical embedded
and automotive software engineering skills — CAN bus mechanics, AUTOSAR
component architecture, diagnostic protocols, and safety-oriented testing
— rather than a single isolated exercise.

**Status: in active development (2026).** This README doubles as the
project roadmap; sections are checked off as they're implemented.

## Architecture

```
automotive-ecu-simulator/
├── include/
│   ├── can_bus/          → Virtual CAN bus (arbitration, framing, pub/sub)
│   ├── ecu_nodes/         → BMS modeled as an AUTOSAR-style software component
│   ├── diagnostics/       → UDS diagnostic services
│   └── fault_injection/   → Fault injection layer + fail-safe verification
├── src/                   → Application entry point / demo driver
├── tests/                 → Unit tests (Google Test)
├── tools/                 → Python test harness, codec, and CLI decoder
└── .github/workflows/     → CI (build + test on push)
```

The [bit-protocol-parser](https://github.com/HusejnG/bit-protocol-parser)
library handles CAN/UDS frame encoding for this project rather than
duplicating bit-packing logic here — pulled in directly via CMake
`FetchContent` as a git dependency.

## Roadmap

- [x] **Virtual CAN bus** — arbitration by message ID priority, frame
      structure, publish/subscribe delivery to multiple ECU nodes
- [x] **BMS ECU node** — AUTOSAR-style software component with a state
      machine (Sleep, Charging, Discharging, Balancing, Fault), sending
      status frames over the virtual bus
- [x] **UDS diagnostics** — `DiagnosticSessionControl`,
      `ReadDataByIdentifier`, and a `SecurityAccess` seed-key exchange
- [x] **Fault injection layer** — simulated sensor dropout / bus message
      loss, verifying the BMS transitions to a fail-safe state correctly
- [x] **Python tooling** — test automation harness and log parsing/visualization
- [x] **Unit tests** covering each module so far (Google Test)
- [x] **CI** — build + test on every push (Linux & Windows)

## AUTOSAR simulation — what this does and doesn't model

The BMS node is built to mirror how a real AUTOSAR Software Component
(SWC) works: it never talks to anything directly, only through a defined
port (here, `onFrameReceived()` in and a status frame out), and
`VirtualCanBus` stands in for the RTE that would normally route signals
between components.

One thing I'm deliberately not attempting: in a real AUTOSAR toolchain,
the RTE glue code between components is *generated* from a configuration
description, not hand-written. Building an actual RTE code generator is
a substantial undertaking on its own — since I'm still building up my
AUTOSAR knowledge, that felt like a good project for later once I've
worked with the real tooling, rather than something to bolt on here just
to check a box. `VirtualCanBus` captures the architectural idea (loose
coupling through ports, not direct calls) without the code-generation
machinery behind it.

## UDS diagnostics

`UdsServer` implements three ISO 14229 services against the BMS node:

- **`0x10` DiagnosticSessionControl** — switch between Default/Programming/
  Extended sessions
- **`0x22` ReadDataByIdentifier** — read live BMS values (pack voltage,
  current, state of charge, temperature) by a 2-byte data identifier,
  plus a standard-ish `0xF186` "active session" DID
- **`0x27` SecurityAccess** — seed/key challenge-response. The seed/key
  transform here is a simple, deterministic XOR (`key = seed ^ 0xA5A5`),
  chosen to demonstrate the challenge-response *mechanism* clearly, not
  as a real security boundary — production seed/key algorithms are
  OEM-proprietary and considerably more involved.

Sample output from the demo driver, running a full tester sequence
against a live BMS:

```
=== UDS diagnostic tester sequence ===
-> DiagnosticSessionControl (Extended)
  bus delivered frame id=0x7a0 data=10 03
  bus delivered frame id=0x7a8 data=50 03
-> SecurityAccess: request seed
  bus delivered frame id=0x7a0 data=27 01
  bus delivered frame id=0x7a8 data=67 01 21 11
-> SecurityAccess: send key (computed from seed 0x2111)
  bus delivered frame id=0x7a0 data=27 02 84 b4
  bus delivered frame id=0x7a8 data=67 02
   security unlocked: yes
-> ReadDataByIdentifier: pack voltage (DID 0x1001)
  bus delivered frame id=0x7a0 data=22 10 01
  bus delivered frame id=0x7a8 data=62 10 01 96 64
-> ReadDataByIdentifier: active session (DID 0xF186)
  bus delivered frame id=0x7a0 data=22 f1 86
  bus delivered frame id=0x7a8 data=62 f1 86 03
```

The voltage response (`96 64` = 0x9664 = 38500 raw, ×0.01 = 385.00V)
matches the BMS's actual charging voltage at that point in the demo —
confirming the UDS server is reading a live value out of the BMS node,
not a hardcoded one.

### A bug this module surfaced in the CAN bus itself

Wiring up request/response services exposed a real bug in
`VirtualCanBus::process()`: when a subscriber's `onFrameReceived()`
callback called `bus.send()` to publish a response — a diagnostic server
answering a request while the bus was still iterating over the batch
that request came from — the response frame got pushed onto the same
`pending_` vector currently being iterated. That's undefined behavior
(the loop could read a reallocated/invalidated vector), and it did in
fact segfault one of the UDS tests before the fix.

The fix: `process()` now swaps `pending_` into a local vector *before*
iterating, so anything a callback sends during processing lands in a
fresh queue and is delivered on the *next* `process()` call instead of
corrupting the current one. This also happens to match real CAN
behavior more closely — a response is a new arbitration cycle, not an
instantaneous echo of the request. The CAN bus tests from the previous
module didn't catch this because nothing in that module ever sent a
frame from inside a receive callback; it took a request/response
service to surface it.

## BMS state machine

```
Sleep --(current flows)--> Charging / Discharging
Charging --(SoC reaches 100%)--> Balancing
Balancing --(current stops)--> Sleep
Discharging --(current stops)--> Sleep
ANY state --(over-temp / voltage out of range)--> Fault
Fault --(readings back in range)--> Sleep
Discharging --(SoC critically low)--> Fault   (over-discharge protection)
```

Sample output from the demo driver (`ecu_simulator`), running a charge
cycle followed by an over-temperature fault scenario:

```
=== Charge cycle ===
  bus delivered frame id=0x200 dlc=6
idle -> state: Sleep
  bus delivered frame id=0x200 dlc=6
charging -> state: Charging
  bus delivered frame id=0x200 dlc=6
near full -> state: Charging
  bus delivered frame id=0x200 dlc=6
full -> state: Balancing
  bus delivered frame id=0x200 dlc=6
current stops -> state: Sleep

=== Fault scenario (over-temperature) ===
  bus delivered frame id=0x200 dlc=6
discharging -> state: Discharging
  bus delivered frame id=0x200 dlc=6
overheating -> state: Fault
  bus delivered frame id=0x200 dlc=6
cooled down -> state: Sleep
```

Each status frame is encoded through `bit-protocol-parser`'s
`SignalSpec`/`encodeFrame` — the same 4-signal, 6-byte layout (pack
voltage, pack current, state of charge, temperature) documented in that
project's README.

## Why these specific pieces

Baden-Württemberg's automotive/embedded industry (Bosch, Mercedes, ZF,
and their suppliers) works daily with CAN, AUTOSAR, and UDS — none of
which are typically covered in a university curriculum. The fault
injection layer specifically reflects that automotive software is
safety-critical: "it works" and "it fails safely when something goes
wrong" are different engineering claims, and this project is built to
demonstrate both.

## Building

Requires a C++17 compiler and CMake 3.16+. No OS-specific code — builds
identically on Linux, macOS, and Windows (MSVC or MinGW).

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release -j
```

Run the demo:
```bash
./build/ecu_simulator              # Linux / macOS
build\Release\ecu_simulator.exe    # Windows (MSVC / Visual Studio)
```

Run the tests:
```bash
cd build
ctest --output-on-failure              # Linux / macOS
ctest --output-on-failure -C Release   # Windows (MSVC / Visual Studio)
```

## Fault injection

Passing tests that only feed the BMS good data prove it works when
nothing goes wrong -- they don't prove it fails *safely* when something
does. Two independent things get corrupted here, deliberately at
different layers of the system:

**`FaultInjector`** sits between the "sensor" and the BMS, corrupting
readings before `updateSensors()` sees them:
- **Out-of-range voltage/temperature** -- values BmsNode's existing range
  checks already catch (covered in the BMS module's own tests; here
  they're driven through the injector instead of hardcoded directly).
- **Stuck sensor** -- freezes on the *first* reading it sees and repeats
  it forever. This one exposed a real gap: a frozen-but-plausible
  reading (e.g. a constant 380V/25°C) doesn't look wrong to a naive
  range check, since nothing about the value itself is out of bounds.
  Only the fact that it never changes is the tell. `BmsNode` now tracks
  how many consecutive ticks a reading stays bit-identical and treats
  five in a row as a fault -- a heuristic (real sensor noise means a
  perfectly constant reading over time is itself suspicious), not a
  certainty, and documented as such in `bms_node.h`.

**`LossyRelay`** sits between the bus and a consumer, deterministically
dropping every Nth frame instead of forwarding it -- simulating the
message loss real CAN wiring produces under electrical noise or a
marginal connection. This tests something different from the injector
above: not whether the BMS detects bad *data*, but whether a downstream
consumer tolerates *missing* data instead of assuming every frame it
expects will arrive.

Demo output -- a stuck sensor reporting a constant, individually
plausible reading, only flagged once it's been frozen for 6 ticks:

```
=== Fault injection: stuck sensor ===
tick 1 (frozen, in-range reading) -> state: Sleep
tick 2 (frozen, in-range reading) -> state: Sleep
tick 3 (frozen, in-range reading) -> state: Sleep
tick 4 (frozen, in-range reading) -> state: Sleep
tick 5 (frozen, in-range reading) -> state: Sleep
tick 6 (frozen, in-range reading) -> state: Fault
```

And a lossy relay dropping every 2nd frame between the bus and a tester:

```
=== Fault injection: lossy CAN relay ===
relay: 4 frames seen, 2 dropped, 2 delivered to tester
```

## Python tooling

The C++ tests exercise classes in isolation with hand-fed inputs. The
Python tooling in `tools/` tests the built program as a black box, which
is closer to how a real test rig validates an ECU — and mirrors the split
you see in automotive work, where the ECU software is C/C++ and the test
tooling around it is Python.

**`run_scenarios.py`** — integration harness. Runs `ecu_simulator --json`
as a subprocess and asserts on the resulting event stream: that the charge
cycle reaches `Balancing` at full SoC, that the seed/key exchange actually
unlocks, that the stuck-sensor fault fires on the *sixth* tick and not
earlier, that the lossy relay drops exactly two of four frames. Exits
non-zero on failure, so it runs in CI alongside `ctest`.

```
$ python3 run_scenarios.py
Captured 26 events from the simulator

[PASS] BMS charge cycle reaches Balancing at full charge (4 checks)
[PASS] BMS status frames decode to the expected physical values (6 checks)
[PASS] UDS DiagnosticSessionControl switches to Extended (2 checks)
[PASS] UDS SecurityAccess seed/key exchange unlocks the ECU (4 checks)
[PASS] UDS ReadDataByIdentifier returns live BMS values (3 checks)
[PASS] Stuck sensor trips a fault only after the streak threshold (7 checks)
[PASS] Lossy relay drops exactly the expected frames (5 checks)

7/7 scenarios passed
```

**`signal_codec.py`** — a second, independent implementation of the frame
decoding, written in Python rather than binding to the C++ code. That's
deliberate: a diagnostic tool is normally a separate program from the ECU
software it talks to, often written by a different team. Two independent
implementations agreeing is a stronger check that the frame format is
specified correctly than calling the same code twice would be. If someone
changes the signal layout on one side only, the integration tests fail —
which is exactly what should happen.

**`decode_frame.py`** — CLI utility for reading raw bus traffic:

```
$ python3 decode_frame.py --bms "94 70 00 00 50 41"
BMS status frame:
  pack_voltage_v       380.00 V
  pack_current_a         0.00 A
  state_of_charge       40.00 %
  temperature_c         25.00 °C

$ python3 decode_frame.py --uds "62 10 01 9c 40"
UDS positive response: ReadDataByIdentifier
  data identifier  0x1001
  pack_voltage_v   400.00 V

$ python3 decode_frame.py --uds "7f 22 12"
UDS negative response:
  service   ReadDataByIdentifier
  rejected  SubFunctionNotSupported
```

### Why the simulator has a `--json` flag

The first version of the harness scraped the human-readable output, which
was fragile — editing a log message would break a test that had nothing to
do with the change. The simulator now emits a structured event stream
under `--json`, keeping the machine-readable interface separate from the
prose. The default output is unchanged.

## Planned extension: real hardware bridge

Right now this is a software-in-the-loop simulation — everything runs
in-process, with no real CAN hardware involved. A natural next step once
the core roadmap is done: bridge `VirtualCanBus` to a real ELM327-based
OBD-II adapter (I have one I've used with FORScan for Ford diagnostics,
plus a couple of generic ones) over a serial connection, so a real
diagnostic tool could talk UDS to this simulator — or so this simulator
could be pointed at data captured from a real vehicle. That would turn
this from a portfolio demo into an actual software-in-the-loop test rig,
which is a real, valued pattern in automotive tooling (testing
diagnostic software without needing real ECU hardware on the bench).
This isn't implemented yet — noting it here as a deliberate next step,
not a finished feature.

## Related projects

- [concurrent-queue-benchmark](https://github.com/HusejnG/concurrent-queue-benchmark) — mutex vs. lock-free queue benchmark
- [bit-protocol-parser](https://github.com/HusejnG/bit-protocol-parser) — bit-level CAN/UDS signal codec, integrated here for frame encoding
