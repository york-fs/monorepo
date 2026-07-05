#include <dti.hh>

#include <can.hh>
#include <util.hh>

#include <cstdint>
#include <optional>

namespace dti {

std::optional<GeneralData1> GeneralData1::decode(util::Stream &stream) {
    const auto erpm = stream.read_be<std::int32_t>();
    const auto duty_cycle = stream.read_be<std::int16_t>();
    const auto input_voltage = stream.read_be<std::int16_t>();
    if (!erpm || !duty_cycle || !input_voltage) {
        return std::nullopt;
    }
    return GeneralData1{
        .erpm = *erpm,
        .duty_cycle = *duty_cycle,
        .input_voltage = *input_voltage,
    };
}

std::optional<GeneralData2> GeneralData2::decode(util::Stream &stream) {
    const auto ac_current = stream.read_be<std::int16_t>();
    const auto dc_current = stream.read_be<std::int16_t>();
    if (!ac_current || !dc_current) {
        return std::nullopt;
    }
    return GeneralData2{
        .ac_current = *ac_current,
        .dc_current = *dc_current,
    };
}

std::optional<GeneralData3> GeneralData3::decode(util::Stream &stream) {
    const auto controller_temperature = stream.read_be<std::int16_t>();
    const auto motor_temperature = stream.read_be<std::int16_t>();
    const auto fault_code = stream.read_byte();
    if (!controller_temperature || !motor_temperature || !fault_code) {
        return std::nullopt;
    }
    return GeneralData3{
        .controller_temperature = *controller_temperature,
        .motor_temperature = *motor_temperature,
        .fault_code = static_cast<FaultCode>(*fault_code),
    };
}

std::optional<GeneralData5> GeneralData5::decode(util::Stream &stream) {
    const auto throttle = stream.read_be<std::int8_t>();
    const auto brake = stream.read_be<std::int8_t>();
    const auto digital_pin_state = stream.read_byte();
    const auto drive_enabled = stream.read_byte();
    const auto limit_flags = stream.read_le<std::uint16_t>();
    stream.read_byte();
    const auto can_map_version = stream.read_byte();
    if (!throttle || !brake || !digital_pin_state || !drive_enabled || !limit_flags || !can_map_version) {
        return std::nullopt;
    }
    return GeneralData5{
        .throttle = *throttle,
        .brake = *brake,
        .digital_pin_state = *digital_pin_state,
        .drive_enabled = *drive_enabled != 0,
        .limit_flags = LimitFlags(*limit_flags),
        .can_map_version = *can_map_version,
    };
}

can::Frame build_set_current(std::uint8_t node_id, std::int16_t current) {
    current = util::clamp(current, -10000, 10000);
    return can::build_extended((0x01u << 8u) | node_id, util::write_be(current));
}

can::Frame build_set_brake_current(std::uint8_t node_id, std::uint16_t current) {
    current = util::clamp(current, 0, 10000);
    return can::build_extended((0x02u << 8u) | node_id, util::write_be(current));
}

can::Frame build_set_erpm(std::uint8_t node_id, std::int32_t erpm) {
    return can::build_extended((0x03u << 8u) | node_id, util::write_be(erpm));
}

can::Frame build_set_position(std::uint8_t node_id, std::int16_t position) {
    return can::build_extended((0x04u << 8u) | node_id, util::write_be(position));
}

can::Frame build_set_relative_current(std::uint8_t node_id, std::int16_t percentage) {
    percentage = util::clamp(percentage, -1000, 1000);
    return can::build_extended((0x05u << 8u) | node_id, util::write_be(percentage));
}

can::Frame build_set_relative_brake_current(std::uint8_t node_id, std::uint16_t percentage) {
    percentage = util::clamp(percentage, 0, 1000);
    return can::build_extended((0x06u << 8u) | node_id, util::write_be(percentage));
}

can::Frame build_set_drive_enabled(std::uint8_t node_id, bool drive_enabled) {
    std::array<std::uint8_t, 1> data{static_cast<std::uint8_t>(drive_enabled ? 1 : 0)};
    return can::build_extended((0x0cu << 8u) | node_id, data);
}

} // namespace dti
