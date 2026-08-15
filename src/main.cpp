// main.cpp
//
// Demo: a BMS node runs through a charge cycle (idle -> charging -> full
// -> balancing -> sleep) and a fault scenario (over-temperature), showing
// the state machine transitions and the status frames published on the
// virtual CAN bus at each step.

#include "can_bus/can_frame.h"
#include "can_bus/virtual_can_bus.h"
#include "ecu_nodes/bms_node.h"

#include <iostream>

namespace {

class LoggingNode : public ECUNode {
public:
    void onFrameReceived(const CanFrame& frame) override {
        std::cout << "  bus delivered frame id=0x" << std::hex << frame.id << std::dec
                  << " dlc=" << static_cast<int>(frame.dlc) << "\n";
    }
};

void runStep(VirtualCanBus& bus, BmsNode& bms, const char* label,
             double voltage, double current, double temperature, double soc) {
    bms.updateSensors(voltage, current, temperature, soc);
    bms.tick();
    bus.process();
    std::cout << label << " -> state: " << toString(bms.state()) << "\n";
}

} // namespace

int main() {
    VirtualCanBus bus;
    LoggingNode logger;
    bus.subscribe(&logger);

    BmsNode bms(bus, 0x200);

    std::cout << "=== Charge cycle ===\n";
    runStep(bus, bms, "idle",         380.0,  0.0, 25.0, 40.0);
    runStep(bus, bms, "charging",     385.0, 10.0, 27.0, 60.0);
    runStep(bus, bms, "near full",    400.0,  8.0, 29.0, 98.0);
    runStep(bus, bms, "full",         400.0,  5.0, 30.0, 100.0);
    runStep(bus, bms, "current stops",400.0,  0.0, 28.0, 100.0);

    std::cout << "\n=== Fault scenario (over-temperature) ===\n";
    runStep(bus, bms, "discharging",  370.0, -20.0, 35.0, 80.0);
    runStep(bus, bms, "overheating",  370.0, -20.0, 75.0, 78.0);
    runStep(bus, bms, "cooled down",  375.0,   0.0, 30.0, 78.0);

    return 0;
}
