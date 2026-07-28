#pragma once

#include <util.hh>

#include <cstdint>

namespace front {

struct ThrottleMessage {
    std::uint16_t desired_current;
    std::uint16_t raw_1;
    std::uint16_t raw_2;

    static constexpr std::uint32_t packet_id() { return 0x1; }
    static constexpr std::uint32_t default_priority() { return 5; }
    static std::optional<ThrottleMessage> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const;
};

} // namespace front
