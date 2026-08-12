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

The [bit-protocol-parser](https://github.com/[your-handle]/bit-protocol-parser)
library handles CAN/UDS frame encoding for this project rather than
duplicating bit-packing logic here.

## Roadmap

- [x] **Virtual CAN bus** — arbitration by message ID priority, frame
      structure, publish/subscribe delivery to multiple ECU nodes
- [ ] **BMS ECU node** — AUTOSAR-style software component with a state
      machine (Charging, Discharging, Fault, Balancing, Sleep), sending
      status frames over the virtual bus
- [ ] **UDS diagnostics** — `DiagnosticSessionControl`,
      `ReadDataByIdentifier`, and a `SecurityAccess` seed-key exchange
- [ ] **Fault injection layer** — simulated sensor dropout / bus message
      loss, verifying the BMS transitions to a fail-safe state correctly
- [ ] **Python tooling** — test automation harness and log parsing/visualization
- [ ] **Unit tests** covering each module (Google Test)
- [ ] **CI** — build + test on every push (Linux & Windows)

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

## Related projects

- [concurrent-queue-benchmark](https://github.com/[your-handle]/concurrent-queue-benchmark) — mutex vs. lock-free queue benchmark
- [bit-protocol-parser](https://github.com/[your-handle]/bit-protocol-parser) — bit-level CAN/UDS signal codec, integrated here for frame encoding
