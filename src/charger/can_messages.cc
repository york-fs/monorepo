#include <charger/can_messages.hh>

#include <charger/error.hh>
#include <util.hh>

#include <cstdint>
#include <optional>

namespace charger {

std::optional<StatusMessage> StatusMessage::decode(util::Stream &stream) {
    const auto error_flags = stream.read_be<ErrorFlags::type_t>();
    const auto charge_voltage = stream.read_be<std::uint16_t>();
    const auto enabled = stream.read_byte();
    if (!error_flags || !charge_voltage || !enabled) {
        return std::nullopt;
    }
    return StatusMessage{
        .error_flags = ErrorFlags(*error_flags),
        .charge_voltage = *charge_voltage,
        .enabled = *enabled == 1,
    };
}

bool StatusMessage::encode(util::Stream &stream) const {
    if (!stream.write_be(error_flags.value())) {
        return false;
    }
    if (!stream.write_be(charge_voltage)) {
        return false;
    }
    return stream.write_byte(enabled ? 1 : 0);
}

std::optional<ControlMessage> ControlMessage::decode(util::Stream &stream) {
    const auto target_current = stream.read_be<std::uint16_t>();
    const auto target_voltage = stream.read_be<std::uint16_t>();
    const auto enable = stream.read_byte();
    if (!target_current || !target_voltage || !enable) {
        return std::nullopt;
    }
    return ControlMessage{
        .target_current = *target_current,
        .target_voltage = *target_voltage,
        .enable = *enable == 1,
    };
}

bool ControlMessage::encode(util::Stream &stream) const {
    if (!stream.write_be(target_current)) {
        return false;
    }
    if (!stream.write_be(target_voltage)) {
        return false;
    }
    return stream.write_byte(enable ? 1 : 0);
}

} // namespace charger
