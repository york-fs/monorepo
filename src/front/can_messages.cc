#include <front/can_messages.hh>

#include <util.hh>

#include <cstdint>
#include <optional>

namespace front {

std::optional<StatusMessage> StatusMessage::decode(util::Stream &stream) {
    const auto ts_activation_desired = stream.read_byte();
    const auto rtd_activation_desired = stream.read_byte();
    if (!ts_activation_desired || !rtd_activation_desired) {
        return std::nullopt;
    }
    return StatusMessage{
        .ts_activation_desired = *ts_activation_desired == 0xaa,
        .rtd_activation_desired = *rtd_activation_desired == 0xaa,
    };
}

bool StatusMessage::encode(util::Stream &stream) const {
    if (!stream.write_byte(ts_activation_desired ? 0xaa : 0)) {
        return false;
    }
    return stream.write_byte(rtd_activation_desired ? 0xaa : 0);
}

std::optional<LvsSampleMessage1> LvsSampleMessage1::decode(util::Stream &stream) {
    const auto rtd_voltage = stream.read_be<std::uint16_t>();
    const auto apps_1_voltage = stream.read_be<std::uint16_t>();
    const auto apps_2_voltage = stream.read_be<std::uint16_t>();
    const auto front_voltage = stream.read_be<std::uint16_t>();
    if (!rtd_voltage || !apps_1_voltage || !apps_2_voltage || !front_voltage) {
        return std::nullopt;
    }
    return LvsSampleMessage1{
        .rtd_voltage = *rtd_voltage,
        .apps_1_voltage = *apps_1_voltage,
        .apps_2_voltage = *apps_2_voltage,
        .front_voltage = *front_voltage,
    };
}

bool LvsSampleMessage1::encode(util::Stream &stream) const {
    if (!stream.write_be(rtd_voltage)) {
        return false;
    }
    if (!stream.write_be(apps_1_voltage)) {
        return false;
    }
    if (!stream.write_be(apps_2_voltage)) {
        return false;
    }
    return stream.write_be(front_voltage);
}

std::optional<LvsSampleMessage2> LvsSampleMessage2::decode(util::Stream &stream) {
    const auto dwin_voltage = stream.read_be<std::uint16_t>();
    const auto aux_1_voltage = stream.read_be<std::uint16_t>();
    const auto aux_2_voltage = stream.read_be<std::uint16_t>();
    if (!dwin_voltage || !aux_1_voltage || !aux_2_voltage) {
        return std::nullopt;
    }
    return LvsSampleMessage2{
        .dwin_voltage = *dwin_voltage,
        .aux_1_voltage = *aux_1_voltage,
        .aux_2_voltage = *aux_2_voltage,
    };
}

bool LvsSampleMessage2::encode(util::Stream &stream) const {
    if (!stream.write_be(dwin_voltage)) {
        return false;
    }
    if (!stream.write_be(aux_1_voltage)) {
        return false;
    }
    return stream.write_be(aux_2_voltage);
}

std::optional<ThrottleMessage> ThrottleMessage::decode(util::Stream &stream) {
    const auto desired_current = stream.read_be<std::uint16_t>();
    const auto raw_1 = stream.read_be<std::uint16_t>();
    const auto raw_2 = stream.read_be<std::uint16_t>();
    if (!desired_current || !raw_1 || !raw_2) {
        return std::nullopt;
    }
    return ThrottleMessage{
        .desired_current = *desired_current,
        .raw_1 = *raw_1,
        .raw_2 = *raw_2,
    };
}

bool ThrottleMessage::encode(util::Stream &stream) const {
    if (!stream.write_be(desired_current)) {
        return false;
    }
    if (!stream.write_be(raw_1)) {
        return false;
    }
    return stream.write_be(raw_2);
}

} // namespace front
