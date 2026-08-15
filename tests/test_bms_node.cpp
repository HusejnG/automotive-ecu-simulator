// test_bms_node.cpp

#include <gtest/gtest.h>

#include "can_bus/virtual_can_bus.h"
#include "ecu_nodes/bms_node.h"

namespace {

// Records every frame the bus delivers, so tests can inspect what the
// BMS actually published.
class RecordingNode : public ECUNode {
public:
    void onFrameReceived(const CanFrame& frame) override {
        lastFrame = frame;
        frameCount++;
    }
    CanFrame lastFrame;
    int frameCount = 0;
};

} // namespace

TEST(BmsNode, StartsInSleepState) {
    VirtualCanBus bus;
    BmsNode bms(bus, 0x200);
    EXPECT_EQ(bms.state(), BmsState::Sleep);
}

TEST(BmsNode, PositiveCurrentTransitionsToCharging) {
    VirtualCanBus bus;
    BmsNode bms(bus, 0x200);

    bms.updateSensors(/*voltage=*/380.0, /*current=*/10.0, /*temp=*/25.0, /*soc=*/50.0);
    bms.tick();

    EXPECT_EQ(bms.state(), BmsState::Charging);
}

TEST(BmsNode, NegativeCurrentTransitionsToDischarging) {
    VirtualCanBus bus;
    BmsNode bms(bus, 0x200);

    bms.updateSensors(380.0, /*current=*/-15.0, 25.0, 50.0);
    bms.tick();

    EXPECT_EQ(bms.state(), BmsState::Discharging);
}

TEST(BmsNode, FullChargeTransitionsToBalancing) {
    VirtualCanBus bus;
    BmsNode bms(bus, 0x200);

    bms.updateSensors(400.0, /*current=*/5.0, 25.0, /*soc=*/100.0);
    bms.tick();

    EXPECT_EQ(bms.state(), BmsState::Balancing);
}

TEST(BmsNode, CurrentStoppingReturnsToSleep) {
    VirtualCanBus bus;
    BmsNode bms(bus, 0x200);

    bms.updateSensors(380.0, 10.0, 25.0, 50.0);
    bms.tick();
    ASSERT_EQ(bms.state(), BmsState::Charging);

    bms.updateSensors(380.0, /*current=*/0.0, 25.0, 50.0);
    bms.tick();
    EXPECT_EQ(bms.state(), BmsState::Sleep);
}

TEST(BmsNode, OverTemperatureTriggersFaultFromAnyState) {
    VirtualCanBus bus;
    BmsNode bms(bus, 0x200);

    bms.updateSensors(380.0, 10.0, 25.0, 50.0);
    bms.tick();
    ASSERT_EQ(bms.state(), BmsState::Charging);

    bms.updateSensors(380.0, 10.0, /*temp=*/75.0, 50.0);
    bms.tick();
    EXPECT_EQ(bms.state(), BmsState::Fault);
}

TEST(BmsNode, OutOfRangeVoltageTriggersFault) {
    VirtualCanBus bus;
    BmsNode bms(bus, 0x200);

    bms.updateSensors(/*voltage=*/-5.0, 0.0, 25.0, 50.0);
    bms.tick();

    EXPECT_EQ(bms.state(), BmsState::Fault);
}

TEST(BmsNode, OverDischargeProtectionTriggersFault) {
    VirtualCanBus bus;
    BmsNode bms(bus, 0x200);

    bms.updateSensors(350.0, /*current=*/-20.0, 25.0, /*soc=*/1.0);
    bms.tick();

    EXPECT_EQ(bms.state(), BmsState::Fault);
}

TEST(BmsNode, RecoversFromFaultWhenReadingsReturnToRange) {
    VirtualCanBus bus;
    BmsNode bms(bus, 0x200);

    bms.updateSensors(380.0, 10.0, 75.0, 50.0); // over-temp
    bms.tick();
    ASSERT_EQ(bms.state(), BmsState::Fault);

    bms.updateSensors(380.0, 0.0, 25.0, 50.0); // back to normal
    bms.tick();
    EXPECT_EQ(bms.state(), BmsState::Sleep);
}

TEST(BmsNode, PublishesStatusFrameOnTick) {
    VirtualCanBus bus;
    RecordingNode recorder;
    bus.subscribe(&recorder);

    BmsNode bms(bus, 0x200);
    bms.updateSensors(364.80, 45.0, 31.0, 62.5);
    bms.tick();
    bus.process();

    ASSERT_EQ(recorder.frameCount, 1);
    EXPECT_EQ(recorder.lastFrame.id, 0x200u);
    EXPECT_EQ(recorder.lastFrame.dlc, 6); // 48 bits = 6 bytes, matches bit-protocol-parser example
}

TEST(BmsNode, StatusFrameRoundTripsCorrectly) {
    // Verifies the BMS's encoded frame can be decoded back to the
    // original sensor readings via the same signal spec used to pack it
    // -- i.e. the ECU simulator and the codec agree on the frame layout.
    VirtualCanBus bus;
    RecordingNode recorder;
    bus.subscribe(&recorder);

    BmsNode bms(bus, 0x200);
    bms.updateSensors(364.80, 45.0, 31.0, 62.5);
    bms.tick();
    bus.process();

    std::vector<SignalSpec> specs = {
        {"pack_voltage_v",  16, false, 0.01, 0.0},
        {"pack_current_a",  16, true,  0.1,  0.0},
        {"state_of_charge",  8, false, 0.5,  0.0},
        {"temperature_c",    8, false, 1.0, -40.0},
    };
    std::vector<std::uint8_t> payload(recorder.lastFrame.data.begin(),
                                       recorder.lastFrame.data.begin() + recorder.lastFrame.dlc);
    auto decoded = decodeFrame(specs, payload);

    EXPECT_NEAR(decoded[0], 364.80, 0.01);
    EXPECT_NEAR(decoded[1], 45.0, 0.1);
    EXPECT_NEAR(decoded[2], 62.5, 0.5);
    EXPECT_NEAR(decoded[3], 31.0, 1.0);
}
