#include <front/can_messages.hh>

#include <util.hh>

#include <cstdint>
#include <optional>

namespace front {

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
