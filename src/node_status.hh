#pragma once

#include <util/stream.hh>

#include <cstdint>
#include <optional>

namespace node_status {

struct NodeStatusMessage1 {
    std::uint32_t can_rx_count;
    std::uint32_t can_tx_count;

    static constexpr std::uint32_t packet_id() { return 0x100; }
    static constexpr std::uint32_t default_priority() { return 6; }
    static std::optional<NodeStatusMessage1> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const;
};

struct NodeStatusMessage2 {
    std::uint32_t can_lost_rx_count;
    std::uint32_t can_lost_tx_count;

    static constexpr std::uint32_t packet_id() { return 0x101; }
    static constexpr std::uint32_t default_priority() { return 6; }
    static std::optional<NodeStatusMessage2> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const;
};

struct NodeStatusMessage3 {
    std::uint32_t uptime_ms;
    std::int8_t mcu_temp;

    static constexpr std::uint32_t packet_id() { return 0x102; }
    static constexpr std::uint32_t default_priority() { return 6; }
    static std::optional<NodeStatusMessage3> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const;
};

void init(std::uint8_t node_id);
void update(std::uint32_t mcu_temp_voltage);

} // namespace node_status
