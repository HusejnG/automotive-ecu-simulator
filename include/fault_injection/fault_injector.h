// fault_injector.h
//
// Deliberately corrupts sensor readings before they reach the BMS, so
// tests can verify the BMS actually detects and reacts to bad data
// rather than trusting it blindly. This is the "does it fail safely"
// half of the safety story that a passing happy-path test can't show.

#pragma once

struct SensorReading {
    double voltage = 0.0;
    double current = 0.0;
    double temperature = 0.0;
    double soc = 0.0;
};

enum class FaultType {
    None,
    StuckSensor,           // sensor freezes on whatever its first reading was
    OutOfRangeVoltage,     // voltage reports a physically implausible value
    OutOfRangeTemperature, // temperature reports a physically implausible value
};

class FaultInjector {
public:
    void injectFault(FaultType type) {
        activeFault_ = type;
        hasFrozenReading_ = false; // start a fresh freeze next time StuckSensor applies
    }

    void clearFault() { activeFault_ = FaultType::None; }

    FaultType activeFault() const { return activeFault_; }

    // Takes a "nominal" (correct) reading and returns what the BMS
    // actually receives -- corrupted according to whatever fault is
    // currently active.
    SensorReading apply(const SensorReading& nominal) {
        switch (activeFault_) {
            case FaultType::StuckSensor:
                if (!hasFrozenReading_) {
                    frozenReading_ = nominal;
                    hasFrozenReading_ = true;
                }
                return frozenReading_;

            case FaultType::OutOfRangeVoltage: {
                SensorReading corrupted = nominal;
                corrupted.voltage = 999.0; // far outside BmsNode's valid range
                return corrupted;
            }

            case FaultType::OutOfRangeTemperature: {
                SensorReading corrupted = nominal;
                corrupted.temperature = 150.0; // far outside BmsNode's valid range
                return corrupted;
            }

            case FaultType::None:
            default:
                return nominal;
        }
    }

private:
    FaultType activeFault_ = FaultType::None;
    SensorReading frozenReading_{};
    bool hasFrozenReading_ = false;
};
