#pragma once

#include <precharge/error.hh>
#include <precharge/relay.hh>
#include <precharge/state.hh>
#include <util/stream.hh>

#include <cstdint>
#include <optional>

namespace precharge {

struct StatusMessage {
    std::uint16_t precharge_voltage;
    std::uint16_t tractive_voltage;
    ErrorFlags error_flags;
    RelayStates relay_states;
    State state;

    static constexpr std::uint32_t packet_id() { return 0x200; }
    static constexpr std::uint32_t default_priority() { return 1; }
    static std::optional<StatusMessage> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const;
};

struct HeartbeatMessage {
    static constexpr std::uint32_t packet_id() { return 0x300; }
    static constexpr std::uint32_t default_priority() { return 3; }
    static std::optional<HeartbeatMessage> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const;
};

} // namespace precharge
