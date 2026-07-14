#pragma once

#include <util.hh>

#include <cstdint>

namespace rear {

struct FlashMessage {
    static constexpr std::uint32_t packet_id() { return 0x1; }
    static constexpr std::uint32_t default_priority() { return 1; }
    static std::optional<FlashMessage> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const;
};

} // namespace rear
