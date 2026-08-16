// main.cpp
//
// Demo: a BMS node runs through a charge cycle (see bms_node.h), and a
// UDS diagnostic server answers a typical tester sequence against it:
// open an extended session, unlock security via a real seed/key
// round trip, then read live BMS values by data identifier.

#include "can_bus/can_frame.h"
#include "can_bus/virtual_can_bus.h"
#include "diagnostics/uds_server.h"
#include "ecu_nodes/bms_node.h"

#include <iomanip>
#include <iostream>

namespace {

// Logs every frame it sees and remembers the last one, so the demo can
// react to responses the way a real diagnostic tester would.
class TesterNode : public ECUNode {
public:
    void onFrameReceived(const CanFrame& frame) override {
        lastFrame = frame;
        std::cout << "  bus delivered frame id=0x" << std::hex << frame.id << std::dec
                   << " data=";
        for (int i = 0; i < frame.dlc; ++i) {
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                       << static_cast<int>(frame.data[i]) << " ";
        }
        std::cout << std::dec << "\n";
    }
    CanFrame lastFrame;
};

void runBmsStep(VirtualCanBus& bus, BmsNode& bms, const char* label,
                double voltage, double current, double temperature, double soc) {
    bms.updateSensors(voltage, current, temperature, soc);
    bms.tick();
    bus.process();
    std::cout << label << " -> state: " << toString(bms.state()) << "\n";
}

} // namespace

int main() {
    VirtualCanBus bus;
    TesterNode tester;
    bus.subscribe(&tester);

    BmsNode bms(bus, 0x200);
    UdsServer uds(bus, 0x7A0, 0x7A8, bms);
    bus.subscribe(&uds);

    std::cout << "=== BMS charge cycle ===\n";
    runBmsStep(bus, bms, "idle",     380.0,  0.0, 25.0, 40.0);
    runBmsStep(bus, bms, "charging", 385.0, 10.0, 27.0, 60.0);

    std::cout << "\n=== UDS diagnostic tester sequence ===\n";

    std::cout << "-> DiagnosticSessionControl (Extended)\n";
    bus.send(CanFrame(0x7A0, {0x10, 0x03}));
    bus.process();
    bus.process(); // deliver the response generated above

    std::cout << "-> SecurityAccess: request seed\n";
    bus.send(CanFrame(0x7A0, {0x27, 0x01}));
    bus.process();
    bus.process(); // deliver the response generated above

    // Pull the seed out of the response the tester just received and
    // compute the matching key -- a real tester would do the same,
    // using whatever the ECU's actual seed/key algorithm is.
    std::uint16_t seed = (static_cast<std::uint16_t>(tester.lastFrame.data[2]) << 8) |
                          tester.lastFrame.data[3];
    std::uint16_t key = seed ^ 0xA5A5;

    std::cout << "-> SecurityAccess: send key (computed from seed 0x"
               << std::hex << seed << ")\n" << std::dec;
    bus.send(CanFrame(0x7A0, {0x27, 0x02,
                               static_cast<std::uint8_t>(key >> 8),
                               static_cast<std::uint8_t>(key & 0xFF)}));
    bus.process();
    bus.process(); // deliver the response generated above
    std::cout << "   security unlocked: " << (uds.securityUnlocked() ? "yes" : "no") << "\n";

    std::cout << "-> ReadDataByIdentifier: pack voltage (DID 0x1001)\n";
    bus.send(CanFrame(0x7A0, {0x22, 0x10, 0x01}));
    bus.process();
    bus.process(); // deliver the response generated above

    std::cout << "-> ReadDataByIdentifier: active session (DID 0xF186)\n";
    bus.send(CanFrame(0x7A0, {0x22, 0xF1, 0x86}));
    bus.process();
    bus.process(); // deliver the response generated above

    return 0;
}
