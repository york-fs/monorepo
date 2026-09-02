#pragma once

#include <front/shutdown.hh>
#include <util/stream.hh>

#include <cstdint>

namespace front {

struct StatusMessage {
    ShutdownSamples shutdown_samples;
    bool ts_activation_desired;
    bool rtd_activation_desired;

    static constexpr std::uint32_t packet_id() { return 0x200; }
    static constexpr std::uint32_t default_priority() { return 1; }
    static std::optional<StatusMessage> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const;
};

struct LvsSampleMessage1 {
    std::uint16_t rtd_voltage;
    std::uint16_t apps_1_voltage;
    std::uint16_t apps_2_voltage;
    std::uint16_t front_voltage;

    static constexpr std::uint32_t packet_id() { return 0x201; }
    static constexpr std::uint32_t default_priority() { return 5; }
    static std::optional<LvsSampleMessage1> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const;
};

struct LvsSampleMessage2 {
    std::uint16_t dwin_voltage;
    std::uint16_t aux_1_voltage;
    std::uint16_t aux_2_voltage;

    static constexpr std::uint32_t packet_id() { return 0x202; }
    static constexpr std::uint32_t default_priority() { return 5; }
    static std::optional<LvsSampleMessage2> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const;
};

struct ThrottleMessage {
    std::uint16_t desired_throttle;
    std::uint16_t pedal_travel;
    std::uint16_t raw_1;
    std::uint16_t raw_2;

    static constexpr std::uint32_t packet_id() { return 0x203; }
    static constexpr std::uint32_t default_priority() { return 2; }
    static std::optional<ThrottleMessage> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const;
};

} // namespace front
