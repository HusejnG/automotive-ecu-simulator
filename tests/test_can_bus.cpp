// test_can_bus.cpp

#include <gtest/gtest.h>

#include "can_bus/can_frame.h"
#include "can_bus/virtual_can_bus.h"

namespace {

// Records every frame it receives, in the order received, so tests can
// assert on delivery order.
class RecordingNode : public ECUNode {
public:
    void onFrameReceived(const CanFrame& frame) override {
        receivedIds.push_back(frame.id);
    }
    std::vector<std::uint32_t> receivedIds;
};

} // namespace

TEST(CanFrame, ConstructsWithCorrectDlcAndData) {
    CanFrame frame(0x123, {0x01, 0x02, 0x03});
    EXPECT_EQ(frame.id, 0x123u);
    EXPECT_EQ(frame.dlc, 3);
    EXPECT_EQ(frame.data[0], 0x01);
    EXPECT_EQ(frame.data[1], 0x02);
    EXPECT_EQ(frame.data[2], 0x03);
}

TEST(CanFrame, TruncatesPayloadLongerThan8Bytes) {
    CanFrame frame(0x1, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10});
    EXPECT_EQ(frame.dlc, 8);
}

TEST(VirtualCanBus, DeliversToAllSubscribers) {
    VirtualCanBus bus;
    RecordingNode a, b;
    bus.subscribe(&a);
    bus.subscribe(&b);

    bus.send(CanFrame(42, {0xFF}));
    bus.process();

    ASSERT_EQ(a.receivedIds.size(), 1u);
    ASSERT_EQ(b.receivedIds.size(), 1u);
    EXPECT_EQ(a.receivedIds[0], 42u);
    EXPECT_EQ(b.receivedIds[0], 42u);
}

TEST(VirtualCanBus, ArbitrationDeliversLowestIdFirst) {
    VirtualCanBus bus;
    RecordingNode node;
    bus.subscribe(&node);

    // Sent out of order on purpose.
    bus.send(CanFrame(500, {}));
    bus.send(CanFrame(10, {}));
    bus.send(CanFrame(200, {}));
    bus.send(CanFrame(1, {}));

    bus.process();

    ASSERT_EQ(node.receivedIds.size(), 4u);
    EXPECT_EQ(node.receivedIds[0], 1u);
    EXPECT_EQ(node.receivedIds[1], 10u);
    EXPECT_EQ(node.receivedIds[2], 200u);
    EXPECT_EQ(node.receivedIds[3], 500u);
}

TEST(VirtualCanBus, ProcessClearsThePendingQueue) {
    VirtualCanBus bus;
    RecordingNode node;
    bus.subscribe(&node);

    bus.send(CanFrame(1, {}));
    EXPECT_EQ(bus.pendingCount(), 1u);

    bus.process();
    EXPECT_EQ(bus.pendingCount(), 0u);

    // A second process() with nothing queued should deliver nothing new.
    bus.process();
    EXPECT_EQ(node.receivedIds.size(), 1u);
}

TEST(VirtualCanBus, EqualIdsPreserveSendOrder) {
    // stable_sort means two frames with the same ID keep their relative
    // send() order -- this matters if, say, two identical-priority
    // status updates are queued in the same cycle.
    VirtualCanBus bus;
    RecordingNode node;
    bus.subscribe(&node);

    CanFrame first(100, {0x01});
    CanFrame second(100, {0x02});
    bus.send(first);
    bus.send(second);
    bus.process();

    ASSERT_EQ(node.receivedIds.size(), 2u);
    EXPECT_EQ(node.receivedIds[0], 100u);
    EXPECT_EQ(node.receivedIds[1], 100u);
}

TEST(VirtualCanBus, NoSubscribersDoesNotCrash) {
    VirtualCanBus bus;
    bus.send(CanFrame(1, {0x01}));
    EXPECT_NO_THROW(bus.process());
}
