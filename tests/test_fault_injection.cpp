// test_fault_injection.cpp

#include <gtest/gtest.h>

#include "can_bus/virtual_can_bus.h"
#include "ecu_nodes/bms_node.h"
#include "fault_injection/fault_injector.h"
#include "fault_injection/lossy_relay.h"

namespace {

class RecordingNode : public ECUNode {
public:
    void onFrameReceived(const CanFrame& frame) override { frames.push_back(frame); }
    std::vector<CanFrame> frames;
};

} // namespace

// ---- FaultInjector: does the BMS actually notice bad data? ----------------

TEST(FaultInjection, NoFaultPassesReadingsThroughUnchanged) {
    FaultInjector injector;
    SensorReading nominal{380.0, 10.0, 25.0, 50.0};

    SensorReading result = injector.apply(nominal);

    EXPECT_EQ(result.voltage, nominal.voltage);
    EXPECT_EQ(result.current, nominal.current);
    EXPECT_EQ(result.temperature, nominal.temperature);
    EXPECT_EQ(result.soc, nominal.soc);
}

TEST(FaultInjection, OutOfRangeVoltageFaultDrivesBmsToFault) {
    VirtualCanBus bus;
    BmsNode bms(bus, 0x200);
    FaultInjector injector;
    injector.injectFault(FaultType::OutOfRangeVoltage);

    SensorReading nominal{380.0, 10.0, 25.0, 50.0};
    SensorReading corrupted = injector.apply(nominal);
    bms.updateSensors(corrupted.voltage, corrupted.current, corrupted.temperature, corrupted.soc);
    bms.tick();

    EXPECT_EQ(bms.state(), BmsState::Fault);
}

TEST(FaultInjection, OutOfRangeTemperatureFaultDrivesBmsToFault) {
    VirtualCanBus bus;
    BmsNode bms(bus, 0x200);
    FaultInjector injector;
    injector.injectFault(FaultType::OutOfRangeTemperature);

    SensorReading nominal{380.0, 10.0, 25.0, 50.0};
    SensorReading corrupted = injector.apply(nominal);
    bms.updateSensors(corrupted.voltage, corrupted.current, corrupted.temperature, corrupted.soc);
    bms.tick();

    EXPECT_EQ(bms.state(), BmsState::Fault);
}

TEST(FaultInjection, StuckSensorEventuallyDrivesBmsToFaultEvenWithPlausibleValues) {
    // The key point of this test: the frozen reading is itself
    // perfectly in-range (nothing alarming about 380V/25C on its own).
    // Only the fact that it never changes should trip the fault -- if
    // this test passes, it's proving BmsNode's staleness detection,
    // not its range checking (which is already covered elsewhere).
    VirtualCanBus bus;
    BmsNode bms(bus, 0x200);
    FaultInjector injector;
    injector.injectFault(FaultType::StuckSensor);

    SensorReading nominal{380.0, 0.0, 25.0, 50.0};

    BmsState lastState = BmsState::Sleep;
    for (int i = 0; i < BmsNode::kStuckSensorStreakThreshold + 1; ++i) {
        SensorReading reading = injector.apply(nominal);
        bms.updateSensors(reading.voltage, reading.current, reading.temperature, reading.soc);
        bms.tick();
        lastState = bms.state();
    }

    EXPECT_EQ(lastState, BmsState::Fault);
}

TEST(FaultInjection, StuckSensorDoesNotFalseTriggerBeforeThreshold) {
    VirtualCanBus bus;
    BmsNode bms(bus, 0x200);
    FaultInjector injector;
    injector.injectFault(FaultType::StuckSensor);

    SensorReading nominal{380.0, 0.0, 25.0, 50.0};

    // One fewer tick than the threshold requires -- should not have
    // triggered the stuck-sensor fault yet.
    for (int i = 0; i < BmsNode::kStuckSensorStreakThreshold - 1; ++i) {
        SensorReading reading = injector.apply(nominal);
        bms.updateSensors(reading.voltage, reading.current, reading.temperature, reading.soc);
        bms.tick();
    }

    EXPECT_NE(bms.state(), BmsState::Fault);
}

TEST(FaultInjection, ClearingFaultAllowsRecoveryOnNextGoodReading) {
    VirtualCanBus bus;
    BmsNode bms(bus, 0x200);
    FaultInjector injector;
    injector.injectFault(FaultType::OutOfRangeTemperature);

    SensorReading nominal{380.0, 0.0, 25.0, 50.0};
    SensorReading corrupted = injector.apply(nominal);
    bms.updateSensors(corrupted.voltage, corrupted.current, corrupted.temperature, corrupted.soc);
    bms.tick();
    ASSERT_EQ(bms.state(), BmsState::Fault);

    injector.clearFault();
    SensorReading good = injector.apply(nominal);
    bms.updateSensors(good.voltage, good.current, good.temperature, good.soc);
    bms.tick();

    EXPECT_EQ(bms.state(), BmsState::Sleep);
}

// ---- LossyRelay: does a consumer tolerate missing frames? -----------------

TEST(LossyRelay, ForwardsAllFramesWhenDropIsDisabled) {
    RecordingNode target;
    LossyRelay relay(target, /*dropEveryNth=*/0);

    for (int i = 0; i < 5; ++i) {
        relay.onFrameReceived(CanFrame(0x1, {static_cast<std::uint8_t>(i)}));
    }

    EXPECT_EQ(target.frames.size(), 5u);
    EXPECT_EQ(relay.droppedCount(), 0);
}

TEST(LossyRelay, DropsEveryThirdFrame) {
    RecordingNode target;
    LossyRelay relay(target, /*dropEveryNth=*/3);

    for (int i = 1; i <= 9; ++i) {
        relay.onFrameReceived(CanFrame(0x1, {static_cast<std::uint8_t>(i)}));
    }

    // Frames 3, 6, 9 dropped -- 6 delivered, 3 dropped.
    EXPECT_EQ(target.frames.size(), 6u);
    EXPECT_EQ(relay.droppedCount(), 3);
    EXPECT_EQ(relay.receivedCount(), 9);
}

TEST(LossyRelay, IntegratesWithBmsStatusFramesOverVirtualCanBus) {
    // End-to-end: BMS publishes status frames on the real bus, a lossy
    // relay sits between the bus and the "tester", and some frames never
    // arrive -- the kind of gap a real diagnostic tool has to tolerate.
    VirtualCanBus bus;
    RecordingNode tester;
    LossyRelay relay(tester, /*dropEveryNth=*/2);
    bus.subscribe(&relay);

    BmsNode bms(bus, 0x200);
    for (int i = 0; i < 4; ++i) {
        bms.updateSensors(380.0 + i, 5.0, 25.0, 50.0 + i);
        bms.tick();
        bus.process();
    }

    EXPECT_EQ(relay.receivedCount(), 4);
    EXPECT_EQ(relay.droppedCount(), 2);   // 2nd and 4th frame
    EXPECT_EQ(tester.frames.size(), 2u);  // 1st and 3rd got through
}
