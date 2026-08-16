// bms_node.h
//
// A Battery Management System modeled as an AUTOSAR-style software
// component: it has no idea who reads its CAN frames or where its sensor
// readings come from, only its own internal state machine and a well
// defined "port" in and out (onFrameReceived / the status frame it sends
// on tick()). VirtualCanBus stands in for the RTE that would normally
// route signals between components.
//
// State machine:
//
//   Sleep --(current != 0)--> Charging / Discharging
//   Charging --(SoC reaches full)--> Balancing
//   Balancing --(current stops)--> Sleep
//   Discharging --(current stops)--> Sleep
//   ANY state --(over-temp / out-of-range voltage)--> Fault
//   Fault --(readings back in range)--> Sleep
//   Discharging --(SoC critically low)--> Fault  (over-discharge protection)

#pragma once

#include "can_bus/can_frame.h"
#include "can_bus/virtual_can_bus.h"
#include "signal_codec.h"

#include <cstdint>

enum class BmsState {
    Sleep,
    Charging,
    Discharging,
    Balancing,
    Fault,
};

inline const char* toString(BmsState state) {
    switch (state) {
        case BmsState::Sleep:        return "Sleep";
        case BmsState::Charging:     return "Charging";
        case BmsState::Discharging:  return "Discharging";
        case BmsState::Balancing:    return "Balancing";
        case BmsState::Fault:        return "Fault";
    }
    return "Unknown";
}

class BmsNode : public ECUNode {
public:
    // statusFrameId is the CAN ID this node publishes its status on.
    BmsNode(VirtualCanBus& bus, std::uint32_t statusFrameId)
        : bus_(bus), statusFrameId_(statusFrameId) {}

    // BMS doesn't currently act on incoming frames (no charger-command
    // handling yet), but implements the port so it can be extended later
    // -- e.g. UDS diagnostic requests targeting this node.
    void onFrameReceived(const CanFrame& /*frame*/) override {}

    // Feeds simulated sensor readings into the component. In a real ECU
    // these would come from ADC drivers; here the caller (main/tests)
    // plays that role.
    void updateSensors(double voltageV, double currentA, double temperatureC, double socPercent) {
        voltage_ = voltageV;
        current_ = currentA;
        temperature_ = temperatureC;
        soc_ = socPercent;
    }

    // Runs one simulation step: evaluates the state machine against the
    // current sensor readings, then publishes a status frame on the bus.
    void tick() {
        evaluateTransitions();
        sendStatusFrame();
    }

    BmsState state() const { return state_; }

    // Exposes current sensor readings for other components (e.g. the UDS
    // server) to read -- mirrors how a real UDS ReadDataByIdentifier
    // handler would pull a live value out of another module.
    double voltage() const { return voltage_; }
    double current() const { return current_; }
    double temperature() const { return temperature_; }
    double stateOfCharge() const { return soc_; }

    static constexpr double kFaultTempC = 60.0;
    static constexpr double kMinVoltageV = 0.0;
    static constexpr double kMaxVoltageV = 500.0;
    static constexpr double kFullSocPercent = 100.0;
    static constexpr double kCriticalLowSocPercent = 2.0;
    static constexpr double kCurrentEpsilonA = 0.05;

private:
    void evaluateTransitions() {
        bool outOfSafeRange = (temperature_ > kFaultTempC) ||
                               (voltage_ < kMinVoltageV) ||
                               (voltage_ > kMaxVoltageV);

        if (outOfSafeRange) {
            state_ = BmsState::Fault;
            return;
        }

        if (state_ == BmsState::Fault) {
            // Readings are back in range -- recover to a safe default
            // state. A real system would typically require an explicit
            // acknowledgment/reset rather than auto-recovering; this is
            // a deliberate simplification for the simulator.
            state_ = BmsState::Sleep;
            return;
        }

        bool isDischarging = current_ < -kCurrentEpsilonA;
        bool isCharging = current_ > kCurrentEpsilonA;

        if (isDischarging && soc_ <= kCriticalLowSocPercent) {
            // Over-discharge protection: treat as a fault rather than
            // letting the pack drain further.
            state_ = BmsState::Fault;
            return;
        }

        if (isCharging) {
            state_ = (soc_ >= kFullSocPercent) ? BmsState::Balancing : BmsState::Charging;
        } else if (isDischarging) {
            state_ = BmsState::Discharging;
        } else {
            // No current flowing -- charging finished, discharging
            // stopped, or balancing complete either way.
            state_ = BmsState::Sleep;
        }
    }

    void sendStatusFrame() {
        static const std::vector<SignalSpec> kBmsSignals = {
            {"pack_voltage_v",  16, false, 0.01, 0.0},
            {"pack_current_a",  16, true,  0.1,  0.0},
            {"state_of_charge",  8, false, 0.5,  0.0},
            {"temperature_c",    8, false, 1.0, -40.0},
        };

        std::vector<double> readings = {voltage_, current_, soc_, temperature_};
        std::vector<std::uint8_t> payload = encodeFrame(kBmsSignals, readings);

        CanFrame frame(statusFrameId_, payload);
        bus_.send(frame);
    }

    VirtualCanBus& bus_;
    std::uint32_t statusFrameId_;

    BmsState state_ = BmsState::Sleep;
    double voltage_ = 0.0;
    double current_ = 0.0;
    double temperature_ = 0.0;
    double soc_ = 0.0;
};
