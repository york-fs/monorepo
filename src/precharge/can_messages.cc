#include <precharge/can_messages.hh>

#include <precharge/error.hh>
#include <precharge/relay.hh>
#include <precharge/state.hh>
#include <util.hh>

#include <cstdint>
#include <optional>
#include <type_traits>

namespace precharge {

std::optional<StatusMessage> StatusMessage::decode(util::Stream &stream) {
    const auto precharge_voltage = stream.read_be<std::uint16_t>();
    const auto tractive_voltage = stream.read_be<std::uint16_t>();
    const auto error_flags = stream.read_be<ErrorFlags::type_t>();
    const auto relay_states = stream.read_be<RelayStates::type_t>();
    const auto state = stream.read_be<std::underlying_type_t<State>>();
    if (!precharge_voltage || !tractive_voltage || !error_flags || !relay_states || !state) {
        return std::nullopt;
    }
    return StatusMessage{
        .precharge_voltage = *precharge_voltage,
        .tractive_voltage = *tractive_voltage,
        .error_flags = ErrorFlags(*error_flags),
        .relay_states = RelayStates(*relay_states),
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
    if (!stream.write_be(error_flags.value())) {
        return false;
    }
    if (!stream.write_be(relay_states.value())) {
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
