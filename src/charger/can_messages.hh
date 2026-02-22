#pragma once

#include <charger/error.hh>
#include <util.hh>

#include <cstdint>
#include <optional>

namespace charger {

struct StatusMessage {
    ErrorFlags error_flags;
    std::uint16_t charge_voltage;
    bool enabled;

    static constexpr std::uint32_t packet_id() { return 0x1; }
    static constexpr std::uint32_t default_priority() { return 5; }
    static std::optional<StatusMessage> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const;
};

struct ControlMessage {
    std::uint16_t target_current;
    std::uint16_t target_voltage;
    bool enable;

    static constexpr std::uint32_t packet_id() { return 0x2; }
    static constexpr std::uint32_t default_priority() { return 4; }
    static std::optional<ControlMessage> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const;
};

} // namespace charger
