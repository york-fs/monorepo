#include <precharge/can_messages.hh>

#include <util.hh>

#include <cstdint>
#include <optional>

namespace precharge {

std::optional<StatusMessage> StatusMessage::decode(util::Stream &stream) {
    const auto precharge_voltage = stream.read_be<std::uint16_t>();
    const auto tractive_voltage = stream.read_be<std::uint16_t>();
    const auto last_error_flags = stream.read_be<ErrorFlags::type_t>();
    const auto state = stream.read_byte();
    if (!precharge_voltage || !tractive_voltage || !last_error_flags || !state) {
        return std::nullopt;
    }
    return StatusMessage{
        .precharge_voltage = *precharge_voltage,
        .tractive_voltage = *tractive_voltage,
        .last_error_flags = ErrorFlags(*last_error_flags),
        .state = State(*state),
    };
}

bool StatusMessage::encode(util::Stream &stream) const {
    if (!stream.write_be(precharge_voltage)) {
        return false;
    }
    if (!stream.write_be(tractive_voltage)) {
        return false;
    }
    if (!stream.write_be(last_error_flags.value())) {
        return false;
    }
    return stream.write_be(util::to_underlying(state));
}

std::optional<ActivateMessage> ActivateMessage::decode(util::Stream &) {
    return ActivateMessage{};
}

bool ActivateMessage::encode(util::Stream &) const {
    return true;
}

} // namespace precharge
