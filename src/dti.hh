#pragma once

#include <cstdint>
#include <variant>

namespace dti {

enum class FaultCode : std::uint8_t {
    NoFaults = 0,
    Overvoltage = 1,
    Undervoltage = 2,
    Drv = 3,
    Overcurrent = 4,
    ControllerOvertemperature = 5,
    MotorOvertemperature = 6,
    SensorWireFault = 7,
    SensorGeneralFault = 8,
    CanCommandError = 9,
    AnalogInputError = 10,
};

struct GeneralData1 {
    // Current electrical ERPM. 1x scale.
    std::int32_t erpm;

    // Current duty cycle. Negative represents regeneration. 10x scale.
    std::int16_t duty_cycle;

    // Measured input DC voltage. 1x scale.
    std::int16_t input_voltage;
};

struct GeneralData2 {
    // Motor current. Negative represents regeneration. 10x scale.
    std::int16_t ac_current;

    // Current on DC side. Negative represents regeneration. 10x scale.
    std::int16_t dc_current;
};

struct GeneralData3 {
    // Current controller temperature. 10x scale.
    std::int16_t controller_temperature;

    // Current motor temperature. 10x scale.
    std::int16_t motor_temperature;

    FaultCode fault_code;
};

struct GeneralData5 {
    // Throttle signal received from analog inputs or CAN2. 1x scale.
    std::int8_t throttle;

    // Brake signal received from analog inputs or CAN2. 1x scale.
    std::int8_t brake;

    // Lower nibble - input states, upper nibble - output states.
    std::uint8_t digital_pin_state;

    // Current drive enable state.
    bool drive_enabled : 1;

    // Various limit activation states.
    bool capacitor_temperature_limit_active : 1;
    bool dc_current_limit_active : 1;
    bool drive_enable_limit_active : 1;
    bool igbt_acceleration_limit_active : 1;
    bool igbt_temperature_limit_active : 1;
    bool input_voltage_limit_active : 1;
    bool motor_acceleration_temperature_limit_active : 1;
    bool motor_temperature_limit_active : 1;
    bool rpm_min_limit_active : 1;
    bool rpm_max_limit_active : 1;
    bool power_limit_active : 1;

    // Configured CAN map version, e.g. 23 or 24.
    std::uint8_t can_map_version;
};

struct UnknownData {
    std::uint32_t data_low;
    std::uint32_t data_high;
};

using Packet = std::variant<GeneralData1, GeneralData2, GeneralData3, GeneralData5, UnknownData>;

Packet parse_packet(std::uint32_t ext_id, std::uint32_t data_low, std::uint32_t data_high);

} // namespace dti
