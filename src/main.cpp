// main.cpp
//
// Demo driver: runs a BMS charge cycle, a UDS diagnostic tester sequence
// against it, and two fault-injection scenarios.
//
// Output modes:
//   (default)  human-readable narration
//   --json     machine-readable event stream, one JSON object per line
//
// The JSON mode exists so the Python test harness in tools/ can assert on
// what actually happened without scraping prose -- changing a log message
// shouldn't break an integration test.

#include "can_bus/can_frame.h"
#include "can_bus/virtual_can_bus.h"
#include "diagnostics/uds_server.h"
#include "ecu_nodes/bms_node.h"
#include "fault_injection/fault_injector.h"
#include "fault_injection/lossy_relay.h"

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace {

bool g_jsonMode = false;

std::string frameDataHex(const CanFrame& frame) {
    std::ostringstream oss;
    for (int i = 0; i < frame.dlc; ++i) {
        if (i > 0) oss << " ";
        oss << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(frame.data[i]);
    }
    return oss.str();
}

void emitFrame(const CanFrame& frame) {
    if (g_jsonMode) {
        std::cout << "{\"event\":\"frame\",\"id\":" << frame.id
                  << ",\"dlc\":" << static_cast<int>(frame.dlc)
                  << ",\"data\":\"" << frameDataHex(frame) << "\"}\n";
    } else {
        std::cout << "  bus delivered frame id=0x" << std::hex << frame.id << std::dec
                  << " data=" << frameDataHex(frame) << "\n";
    }
}

void emitState(const std::string& scenario, const std::string& label, int tick, BmsState state) {
    if (g_jsonMode) {
        std::cout << "{\"event\":\"state\",\"scenario\":\"" << scenario
                  << "\",\"label\":\"" << label
                  << "\",\"tick\":" << tick
                  << ",\"state\":\"" << toString(state) << "\"}\n";
    } else {
        std::cout << label << " -> state: " << toString(state) << "\n";
    }
}

void emitSection(const std::string& name) {
    if (!g_jsonMode) {
        std::cout << "\n=== " << name << " ===\n";
    }
}

void emitStep(const std::string& text) {
    if (!g_jsonMode) {
        std::cout << text << "\n";
    }
}

void emitSecurity(bool unlocked, std::uint16_t seed) {
    if (g_jsonMode) {
        std::cout << "{\"event\":\"security\",\"unlocked\":" << (unlocked ? "true" : "false")
                  << ",\"seed\":" << seed << "}\n";
    } else {
        std::cout << "   security unlocked: " << (unlocked ? "yes" : "no") << "\n";
    }
}

void emitRelayStats(int seen, int dropped, int delivered) {
    if (g_jsonMode) {
        std::cout << "{\"event\":\"relay\",\"seen\":" << seen
                  << ",\"dropped\":" << dropped
                  << ",\"delivered\":" << delivered << "}\n";
    } else {
        std::cout << "relay: " << seen << " frames seen, " << dropped
                  << " dropped, " << delivered << " delivered to tester\n";
    }
}

class TesterNode : public ECUNode {
public:
    void onFrameReceived(const CanFrame& frame) override {
        lastFrame = frame;
        emitFrame(frame);
    }
    CanFrame lastFrame;
};

// A relay target that records frames without emitting them again -- the
// lossy-relay scenario reports aggregate counts instead of each frame.
class SilentNode : public ECUNode {
public:
    void onFrameReceived(const CanFrame&) override {}
};

void runBmsStep(VirtualCanBus& bus, BmsNode& bms, const char* label, int tick,
                double voltage, double current, double temperature, double soc) {
    bms.updateSensors(voltage, current, temperature, soc);
    bms.tick();
    bus.process();
    emitState("charge_cycle", label, tick, bms.state());
}

} // namespace

int main(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--json") g_jsonMode = true;
    }

    VirtualCanBus bus;
    TesterNode tester;
    bus.subscribe(&tester);

    BmsNode bms(bus, 0x200);
    UdsServer uds(bus, 0x7A0, 0x7A8, bms);
    bus.subscribe(&uds);

    emitSection("BMS charge cycle");
    runBmsStep(bus, bms, "idle",      1, 380.0,  0.0, 25.0, 40.0);
    runBmsStep(bus, bms, "charging",  2, 385.0, 10.0, 27.0, 60.0);
    runBmsStep(bus, bms, "near full", 3, 400.0,  8.0, 29.0, 98.0);
    runBmsStep(bus, bms, "full",      4, 400.0,  5.0, 30.0, 100.0);

    emitSection("UDS diagnostic tester sequence");

    emitStep("-> DiagnosticSessionControl (Extended)");
    bus.send(CanFrame(0x7A0, {0x10, 0x03}));
    bus.process();
    bus.process();

    emitStep("-> SecurityAccess: request seed");
    bus.send(CanFrame(0x7A0, {0x27, 0x01}));
    bus.process();
    bus.process();

    std::uint16_t seed = (static_cast<std::uint16_t>(tester.lastFrame.data[2]) << 8) |
                          tester.lastFrame.data[3];
    std::uint16_t key = seed ^ 0xA5A5;

    emitStep("-> SecurityAccess: send key (computed from the seed just received)");
    bus.send(CanFrame(0x7A0, {0x27, 0x02,
                               static_cast<std::uint8_t>(key >> 8),
                               static_cast<std::uint8_t>(key & 0xFF)}));
    bus.process();
    bus.process();
    emitSecurity(uds.securityUnlocked(), seed);

    emitStep("-> ReadDataByIdentifier: pack voltage (DID 0x1001)");
    bus.send(CanFrame(0x7A0, {0x22, 0x10, 0x01}));
    bus.process();
    bus.process();

    emitStep("-> ReadDataByIdentifier: active session (DID 0xF186)");
    bus.send(CanFrame(0x7A0, {0x22, 0xF1, 0x86}));
    bus.process();
    bus.process();

    emitSection("Fault injection: stuck sensor");
    {
        VirtualCanBus faultBus;
        SilentNode sink;
        faultBus.subscribe(&sink);
        BmsNode faultBms(faultBus, 0x200);
        FaultInjector injector;
        injector.injectFault(FaultType::StuckSensor);

        SensorReading nominal{380.0, 0.0, 25.0, 50.0}; // plausible on its own
        for (int i = 1; i <= BmsNode::kStuckSensorStreakThreshold + 1; ++i) {
            SensorReading reading = injector.apply(nominal);
            faultBms.updateSensors(reading.voltage, reading.current,
                                    reading.temperature, reading.soc);
            faultBms.tick();
            faultBus.process();
            std::string label = "tick " + std::to_string(i) + " (frozen, in-range reading)";
            emitState("stuck_sensor", label, i, faultBms.state());
        }
    }

    emitSection("Fault injection: lossy CAN relay");
    {
        VirtualCanBus lossyBus;
        SilentNode lossyTarget;
        LossyRelay relay(lossyTarget, /*dropEveryNth=*/2);
        lossyBus.subscribe(&relay);

        BmsNode lossyBms(lossyBus, 0x200);
        for (int i = 0; i < 4; ++i) {
            lossyBms.updateSensors(380.0 + i, 5.0, 25.0, 50.0 + i);
            lossyBms.tick();
            lossyBus.process();
        }
        emitRelayStats(relay.receivedCount(), relay.droppedCount(),
                        relay.receivedCount() - relay.droppedCount());
    }

    return 0;
}
