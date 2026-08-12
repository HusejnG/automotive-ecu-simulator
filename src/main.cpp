// main.cpp
//
// Demo: three "nodes" queue frames with different IDs in the same cycle,
// a logging node subscribes and prints delivery order -- showing that
// the bus delivers lowest ID first regardless of send() call order.

#include "can_bus/can_frame.h"
#include "can_bus/virtual_can_bus.h"

#include <iostream>

class LoggingNode : public ECUNode {
public:
    void onFrameReceived(const CanFrame& frame) override {
        std::cout << "  node received frame id=" << frame.id
                  << " dlc=" << static_cast<int>(frame.dlc) << "\n";
    }
};

int main() {
    VirtualCanBus bus;
    LoggingNode logger;
    bus.subscribe(&logger);

    // Queued out of ID order on purpose -- process() should still
    // deliver id=10 before id=200 before id=500.
    bus.send(CanFrame(500, {0xAA}));
    bus.send(CanFrame(10, {0xBB}));
    bus.send(CanFrame(200, {0xCC}));

    std::cout << "Pending before process(): " << bus.pendingCount() << "\n";
    std::cout << "Delivery order (lowest ID first):\n";
    bus.process();
    std::cout << "Pending after process(): " << bus.pendingCount() << "\n";

    return 0;
}
