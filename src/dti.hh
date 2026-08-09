#pragma once

#include <util.hh>

#include <cstdint>
#include <optional>

namespace can {

struct Frame;

} // namespace can

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

enum class Limit : std::uint16_t {
    CapacitorTemperature,
    DcCurrent,
    DriveEnable,
    IgbtAcceleration,
    IgbtTemperature,
    InputVoltage,
    MotorAccelerationTemperature,
    MotorTemperature,
    RpmMinimum,
    RpmMaximum,
    PowerLimit,
};

using LimitFlags = util::FlagBitset<Limit>;

struct GeneralData1 {
    // Current electrical ERPM. 1x scale.
    std::int32_t erpm;

    // Current duty cycle. Negative represents regeneration. 10x scale.
    std::int16_t duty_cycle;

    // Measured input DC voltage. 1x scale.
    std::int16_t input_voltage;

    static constexpr std::uint32_t packet_id() { return 0x20; }
    static constexpr std::uint32_t default_priority() { return 0; }
    static std::optional<GeneralData1> decode(util::Stream &stream);
};

struct GeneralData2 {
    // Motor current. Negative represents regeneration. 10x scale.
    std::int16_t ac_current;

    // Current on DC side. Negative represents regeneration. 10x scale.
    std::int16_t dc_current;

    static constexpr std::uint32_t packet_id() { return 0x21; }
    static constexpr std::uint32_t default_priority() { return 0; }
    static std::optional<GeneralData2> decode(util::Stream &stream);
};

struct GeneralData3 {
    // Current controller temperature. 10x scale.
    std::int16_t controller_temperature;

    // Current motor temperature. 10x scale.
    std::int16_t motor_temperature;

    FaultCode fault_code;

    static constexpr std::uint32_t packet_id() { return 0x22; }
    static constexpr std::uint32_t default_priority() { return 0; }
    static std::optional<GeneralData3> decode(util::Stream &stream);
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

    // Limit activation states.
    LimitFlags limit_flags;

    // Configured CAN map version, e.g. 23 or 24.
    std::uint8_t can_map_version;

    static constexpr std::uint32_t packet_id() { return 0x24; }
    static constexpr std::uint32_t default_priority() { return 0; }
    static std::optional<GeneralData5> decode(util::Stream &stream);
};

struct SetCurrentMessage {
    // Absolute AC motor current in hundreds of milliamps. The sign specifies the motor direction.
    std::int16_t current;

    static constexpr std::uint32_t packet_id() { return 0x01; }
    static constexpr std::uint32_t default_priority() { return 0; }
    bool encode(util::Stream &stream) const;
};

struct SetBrakeCurrentMessage {
    // Absolute AC motor brake current in hundreds of milliamps. Unsigned as the inverter automatically applies the
    // current opposite to the current motor direction.
    std::uint16_t current;

    static constexpr std::uint32_t packet_id() { return 0x02; }
    static constexpr std::uint32_t default_priority() { return 0; }
    bool encode(util::Stream &stream) const;
};

struct SetSpeedMessage {
    // Desired speed in ERPM. The sign specifies the motor direction.
    std::int32_t erpm;

    static constexpr std::uint32_t packet_id() { return 0x03; }
    static constexpr std::uint32_t default_priority() { return 0; }
    bool encode(util::Stream &stream) const;
};

struct SetRelativeCurrentMessage {
    // Relative AC motor current in tenths of a percent. The sign specifies the motor direction.
    std::int16_t percentage;

    static constexpr std::uint32_t packet_id() { return 0x05; }
    static constexpr std::uint32_t default_priority() { return 0; }
    bool encode(util::Stream &stream) const;
};

struct SetRelativeBrakeCurrentMessage {
    // Relative AC motor brake current in tenths of a percent. Unsigned as the inverter automatically applies the
    // current opposite to the current motor direction.
    std::int16_t percentage;

    static constexpr std::uint32_t packet_id() { return 0x06; }
    static constexpr std::uint32_t default_priority() { return 0; }
    bool encode(util::Stream &stream) const;
};

struct SetMaxDirectCurrentMessage {
    // Maximum allowable current draw on the DC battery side in hundreds of milliamps.
    std::uint16_t current;

    static constexpr std::uint32_t packet_id() { return 0x0a; }
    static constexpr std::uint32_t default_priority() { return 0; }
    bool encode(util::Stream &stream) const;
};

struct SetMaxBrakeDirectCurrentMessage {
    // Maximum allowable charge current on the DC battery side in hundreds of milliamps.
    std::uint16_t current;

    static constexpr std::uint32_t packet_id() { return 0x0b; }
    static constexpr std::uint32_t default_priority() { return 0; }
    bool encode(util::Stream &stream) const;
};

struct SetDriveEnableMessage {
    bool drive_enable;

    static constexpr std::uint32_t packet_id() { return 0x0c; }
    static constexpr std::uint32_t default_priority() { return 0; }
    bool encode(util::Stream &stream) const;
};

} // namespace dti
