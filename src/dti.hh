#pragma once

#include <cstdint>
#include <variant>

namespace can {

struct Message;

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

struct UnknownMessageType {
    std::uint32_t packet_id;
};

using Packet = std::variant<GeneralData1, GeneralData2, GeneralData3, GeneralData5, UnknownMessageType>;

/**
 * Builds a CAN message for the specified DTI inverter to set the absolute motor current. The value is in hundreds of
 * milliamps and its sign specifies the motor direction.
 *
 * @param node_id the target inverter's node id on the CAN bus
 * @param current an integer in the range [-10000, 10000]
 * @return the built CAN message
 */
can::Message build_set_current(std::uint8_t node_id, std::int16_t current);

/**
 * Builds a CAN message for the specified DTI inverter to set the absolute motor brake current. The value is in hundreds
 * of milliamps. This tells the inverter to apply a current opposite to the current direction of the motor.
 *
 * @param node_id the target inverter's node id on the CAN bus
 * @param current an integer in the range [0, 10000]
 * @return the built CAN message
 */
can::Message build_set_brake_current(std::uint8_t node_id, std::uint16_t current);

/**
 * Builds a CAN message for the specified DTI inverter to set the target ERPM of the inverter's speed control loop. The
 * value is absolute units of ERPM where ERPM is the product of RPM and the number of pole pairs on the motor. The
 * value's sign specifies the motor direction.
 *
 * @param node_id the target inverter's node id on the CAN bus
 * @param erpm the desired ERPM
 * @return the built CAN message
 */
can::Message build_set_erpm(std::uint8_t node_id, std::int32_t erpm);

/**
 * Builds a CAN message for the specified DTI inverter to set an absolute position for the motor to hold. The value is
 * in tenths of a degree.
 *
 * @param node_id the target inverter's node id on the CAN bus
 * @param position the desired position
 * @return the built CAN message
 */
can::Message build_set_position(std::uint8_t node_id, std::int16_t position);

/**
 * Builds a CAN message for the specified DTI inverter to set the relative motor current. The value is in tenths of a
 * percent and its sign specifies the motor direction.
 *
 * @param node_id the target inverter's node id on the CAN bus
 * @param percentage an integer in the range [-1000, 1000]
 * @return the built CAN message
 */
can::Message build_set_relative_current(std::uint8_t node_id, std::int16_t percentage);

/**
 * Builds a CAN message for the specified DTI inverter to set the relative motor brake current. The value is in tenths
 * of a percent.
 *
 * @param node_id the target inverter's node id on the CAN bus
 * @param percentage an integer in the range [0, 1000]
 * @return the built CAN message
 */
can::Message build_set_relative_brake_current(std::uint8_t node_id, std::uint16_t percentage);

/**
 * Builds a CAN message for the specified DTI inverter to set the drive enabled status.
 *
 * @param node_id the target inverter's node id on the CAN bus
 * @param drive_enabled the desired drive enabled status
 * @return the built CAN message
 */
can::Message build_set_drive_enabled(std::uint8_t node_id, bool drive_enabled);

/**
 * Attempts to parse the given CAN message into a DTI status message.
 *
 * @param message the CAN message to parse
 * @return GeneralData if successful
 * @return UnkownMessageType if the packet ID is not known
 */
Packet parse_packet(const can::Message &message);

} // namespace dti
