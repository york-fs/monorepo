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

bool SetCurrentMessage::encode(util::Stream &stream) const {
    return stream.write_be(current);
}

bool SetBrakeCurrentMessage::encode(util::Stream &stream) const {
    return stream.write_be(current);
}

bool SetSpeedMessage::encode(util::Stream &stream) const {
    return stream.write_be(erpm);
}

bool SetRelativeCurrentMessage::encode(util::Stream &stream) const {
    return stream.write_be(util::clamp(percentage, -1000, 1000));
}

bool SetRelativeBrakeCurrentMessage::encode(util::Stream &stream) const {
    return stream.write_be(util::clamp(percentage, 0, 1000));
}

bool SetMaxDirectCurrentMessage::encode(util::Stream &stream) const {
    return stream.write_be(current);
}

bool SetMaxBrakeDirectCurrentMessage::encode(util::Stream &stream) const {
    return stream.write_be(-static_cast<std::int16_t>(current));
}

bool SetDriveEnableMessage::encode(util::Stream &stream) const {
    return stream.write_byte(drive_enable ? 1 : 0);
}

} // namespace dti
