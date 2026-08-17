// lossy_relay.h
//
// Sits between the bus and a real subscriber, deterministically dropping
// every Nth frame instead of forwarding it -- simulating the kind of bus
// error / noise-induced message loss that shows up in real vehicle
// wiring harnesses. Deterministic (not random) on purpose, so tests are
// reproducible: real fault injection in safety testing needs exact,
// repeatable scenarios, not "usually happens sometimes."
//
// This tests something different from BmsNode's own fault detection: it
// exercises whether a *downstream consumer* tolerates missing frames
// gracefully instead of assuming delivery is guaranteed.

#pragma once

#include "can_bus/can_frame.h"
#include "can_bus/virtual_can_bus.h"

class LossyRelay : public ECUNode {
public:
    // dropEveryNth == 3 means the 3rd, 6th, 9th, ... frame is dropped.
    // dropEveryNth <= 0 disables dropping (relay behaves transparently).
    LossyRelay(ECUNode& target, int dropEveryNth)
        : target_(target), dropEveryNth_(dropEveryNth) {}

    void onFrameReceived(const CanFrame& frame) override {
        ++receivedCount_;
        if (dropEveryNth_ > 0 && receivedCount_ % dropEveryNth_ == 0) {
            ++droppedCount_;
            return; // simulated loss -- target never sees this one
        }
        target_.onFrameReceived(frame);
    }

    int receivedCount() const { return receivedCount_; }
    int droppedCount() const { return droppedCount_; }

private:
    ECUNode& target_;
    int dropEveryNth_;
    int receivedCount_ = 0;
    int droppedCount_ = 0;
};
