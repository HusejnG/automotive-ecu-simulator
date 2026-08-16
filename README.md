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
├── tools/                 → Python test automation and log tooling
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
- [ ] **Fault injection layer** — simulated sensor dropout / bus message
      loss, verifying the BMS transitions to a fail-safe state correctly
- [ ] **Python tooling** — test automation harness and log parsing/visualization
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

## Related projects

- [concurrent-queue-benchmark](https://github.com/HusejnG/concurrent-queue-benchmark) — mutex vs. lock-free queue benchmark
- [bit-protocol-parser](https://github.com/HusejnG/bit-protocol-parser) — bit-level CAN/UDS signal codec, integrated here for frame encoding
