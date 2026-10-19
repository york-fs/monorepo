#pragma once

#include <util/flag_bitset.hh>
#include <util/stream.hh>

#include <cstdint>
#include <optional>

namespace elcon {

enum class Status : std::uint8_t {
    HardwareFailure,
    Overtemperature,
    InputVoltageFault,
    BatteryNotDetected,
    CommunicationFault,
};

// status
// 110 00 11111111 01010000 11100101
// 110 00111111 1101010000 11100101
// 0x6 0x3f 0x350 0xe5

// control
// 110 00 00000110 11100101 11110100
// 110 00000001 1011100101 11110100
// 0x6 0x1 0x2e5 0xf4

using StatusFlags = util::FlagBitset<Status>;

struct StatusMessage {
    std::uint16_t output_voltage;
    std::uint16_t output_current;
    StatusFlags status_flags;

    static constexpr std::uint32_t packet_id() { return 0x350; }
    static constexpr std::uint32_t default_priority() { return 6; }
    static constexpr std::uint8_t sub_id() { return 0x3f; }
    static std::optional<StatusMessage> decode(util::Stream &stream, std::uint8_t sub_id);
};

struct ControlMessage {
    std::uint16_t target_voltage;
    std::uint16_t target_current;
    bool disable;

    static constexpr std::uint32_t packet_id() { return 0x2e5; }
    static constexpr std::uint32_t default_priority() { return 6; }
    static constexpr std::uint8_t sub_id() { return 0x1; }
    bool encode(util::Stream &stream) const;
};

} // namespace elcon
