// virtual_can_bus.h
//
// Simulates a CAN bus at the message level: nodes subscribe to the bus,
// send() queues a frame, and process() delivers all pending frames to
// every subscriber, lowest ID first -- mirroring the real CAN rule that
// a lower ID wins arbitration.
//
// This models arbitration's *outcome* (priority ordering when multiple
// frames are pending at once), not the bit-level contention that produces
// it on a real bus. Bit-level arbitration only matters if you're building
// a CAN transceiver; from a software/ECU-logic point of view, the thing
// that actually matters is "which message gets delivered first when
// several are ready at the same time" -- which is exactly what this
// models.

#pragma once

#include "can_frame.h"

#include <algorithm>
#include <cstddef>
#include <vector>

// Nodes on the bus implement this to receive frames. A node still has to
// look at frame.id itself and decide whether it cares -- the bus
// broadcasts to everyone, same as real CAN.
class ECUNode {
public:
    virtual ~ECUNode() = default;
    virtual void onFrameReceived(const CanFrame& frame) = 0;
};

class VirtualCanBus {
public:
    void subscribe(ECUNode* node) {
        subscribers_.push_back(node);
    }

    // Queues a frame for delivery on the next process() call. Multiple
    // send() calls before a process() simulate multiple nodes with
    // frames ready at the same time.
    void send(const CanFrame& frame) {
        pending_.push_back(frame);
    }

    // Sorts pending frames by ID (lower ID = higher priority, same as
    // real CAN arbitration) and delivers each to every subscriber in
    // that order, then clears the queue.
    //
    // Frames sent by a subscriber's onFrameReceived() *during* this call
    // (e.g. a diagnostic server responding to a request) are safe to
    // queue, but are deliberately deferred to the *next* process() call
    // rather than delivered within this same pass -- swapping pending_
    // out before iterating means new sends land in a fresh, empty queue
    // instead of the one currently being iterated (which would otherwise
    // invalidate iterators and corrupt the queue). This also mirrors
    // real CAN behavior: a response is a new arbitration cycle, not an
    // instantaneous echo of the request.
    void process() {
        std::vector<CanFrame> batch;
        batch.swap(pending_);

        std::stable_sort(batch.begin(), batch.end(),
            [](const CanFrame& a, const CanFrame& b) { return a.id < b.id; });

        for (const auto& frame : batch) {
            for (auto* node : subscribers_) {
                node->onFrameReceived(frame);
            }
        }
    }

    std::size_t pendingCount() const { return pending_.size(); }
    std::size_t subscriberCount() const { return subscribers_.size(); }

private:
    std::vector<ECUNode*> subscribers_;
    std::vector<CanFrame> pending_;
};
