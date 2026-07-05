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
    static constexpr std::uint32_t default_priority() { return 7; }
    static std::optional<GeneralData1> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const { return false; }
};

struct GeneralData2 {
    // Motor current. Negative represents regeneration. 10x scale.
    std::int16_t ac_current;

    // Current on DC side. Negative represents regeneration. 10x scale.
    std::int16_t dc_current;

    static constexpr std::uint32_t packet_id() { return 0x21; }
    static constexpr std::uint32_t default_priority() { return 7; }
    static std::optional<GeneralData2> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const { return false; }
};

struct GeneralData3 {
    // Current controller temperature. 10x scale.
    std::int16_t controller_temperature;

    // Current motor temperature. 10x scale.
    std::int16_t motor_temperature;

    FaultCode fault_code;

    static constexpr std::uint32_t packet_id() { return 0x22; }
    static constexpr std::uint32_t default_priority() { return 7; }
    static std::optional<GeneralData3> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const { return false; }
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
    static constexpr std::uint32_t default_priority() { return 7; }
    static std::optional<GeneralData5> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const { return false; }
};

/**
 * Builds a CAN frame for the specified DTI inverter to set the absolute motor current. The value is in hundreds of
 * milliamps and its sign specifies the motor direction.
 *
 * @param node_id the target inverter's node id on the CAN bus
 * @param current an integer in the range [-10000, 10000]
 * @return the built CAN frame
 */
can::Frame build_set_current(std::uint8_t node_id, std::int16_t current);

/**
 * Builds a CAN frame for the specified DTI inverter to set the absolute motor brake current. The value is in hundreds
 * of milliamps. This tells the inverter to apply a current opposite to the current direction of the motor.
 *
 * @param node_id the target inverter's node id on the CAN bus
 * @param current an integer in the range [0, 10000]
 * @return the built CAN frame
 */
can::Frame build_set_brake_current(std::uint8_t node_id, std::uint16_t current);

/**
 * Builds a CAN frame for the specified DTI inverter to set the target ERPM of the inverter's speed control loop. The
 * value is absolute units of ERPM where ERPM is the product of RPM and the number of pole pairs on the motor. The
 * value's sign specifies the motor direction.
 *
 * @param node_id the target inverter's node id on the CAN bus
 * @param erpm the desired ERPM
 * @return the built CAN frame
 */
can::Frame build_set_erpm(std::uint8_t node_id, std::int32_t erpm);

/**
 * Builds a CAN frame for the specified DTI inverter to set an absolute position for the motor to hold. The value is
 * in tenths of a degree.
 *
 * @param node_id the target inverter's node id on the CAN bus
 * @param position the desired position
 * @return the built CAN frame
 */
can::Frame build_set_position(std::uint8_t node_id, std::int16_t position);

/**
 * Builds a CAN frame for the specified DTI inverter to set the relative motor current. The value is in tenths of a
 * percent and its sign specifies the motor direction.
 *
 * @param node_id the target inverter's node id on the CAN bus
 * @param percentage an integer in the range [-1000, 1000]
 * @return the built CAN frame
 */
can::Frame build_set_relative_current(std::uint8_t node_id, std::int16_t percentage);

/**
 * Builds a CAN frame for the specified DTI inverter to set the relative motor brake current. The value is in tenths
 * of a percent.
 *
 * @param node_id the target inverter's node id on the CAN bus
 * @param percentage an integer in the range [0, 1000]
 * @return the built CAN frame
 */
can::Frame build_set_relative_brake_current(std::uint8_t node_id, std::uint16_t percentage);

/**
 * Builds a CAN frame for the specified DTI inverter to set the drive enabled status.
 *
 * @param node_id the target inverter's node id on the CAN bus
 * @param drive_enabled the desired drive enabled status
 * @return the built CAN frame
 */
can::Frame build_set_drive_enabled(std::uint8_t node_id, bool drive_enabled);

} // namespace dti
