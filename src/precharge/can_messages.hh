#pragma once

#include <precharge/error.hh>
#include <precharge/state.hh>
#include <util.hh>

#include <optional>

namespace precharge {

struct StatusMessage {
    std::uint16_t precharge_voltage;
    std::uint16_t tractive_voltage;
    ErrorFlags last_error_flags;
    State state;

    static constexpr std::uint32_t packet_id() { return 0x200; }
    static constexpr std::uint32_t default_priority() { return 1; }
    static std::optional<StatusMessage> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const;
};

struct ActivateMessage {
    static constexpr std::uint32_t packet_id() { return 0x300; }
    static constexpr std::uint32_t default_priority() { return 3; }
    static std::optional<ActivateMessage> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const;
};

} // namespace precharge
