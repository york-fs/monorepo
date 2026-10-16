#pragma once

#include <bms/error.hh>
#include <util/stream.hh>

#include <array>
#include <cstdint>
#include <optional>

namespace bms {

struct MasterStatusMessage {
    MasterErrorFlags master_flags;
    std::uint32_t i2c_error_count;

    static constexpr std::uint32_t packet_id() { return 0x200; }
    static constexpr std::uint32_t default_priority() { return 1; }
    static std::optional<MasterStatusMessage> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const;
};

struct MasterCurrentMessage {
    std::int32_t positive_current;
    std::int32_t negative_current;

    static constexpr std::uint32_t packet_id() { return 0x201; }
    static constexpr std::uint32_t default_priority() { return 2; }
    static std::optional<MasterCurrentMessage> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const;
};

struct MasterSummaryMessage {
    std::uint16_t min_voltage;
    std::uint16_t max_voltage;
    std::int8_t min_temperature;
    std::int8_t max_temperature;

    static constexpr std::uint32_t packet_id() { return 0x202; }
    static constexpr std::uint32_t default_priority() { return 3; }
    static std::optional<MasterSummaryMessage> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const;
};

struct SegmentMessage {
    std::uint8_t segment_id;
    std::uint8_t group_id;

    std::uint8_t sub_id() const { return (segment_id << 2) | (group_id & 0b11u); }
};

struct SegmentSummaryMessage : SegmentMessage {
    SegmentErrorFlags segment_flags;
    std::uint16_t degraded_bitset;
    std::uint16_t balance_bitset;

    static constexpr std::uint32_t packet_id() { return 0x250; }
    static constexpr std::uint32_t default_priority() { return 4; }
    static std::optional<SegmentSummaryMessage> decode(util::Stream &stream, std::uint8_t sub_id);
    bool encode(util::Stream &stream) const;
};

struct SegmentVoltageMessage : SegmentMessage {
    std::array<std::uint16_t, 4> voltages;

    static constexpr std::uint32_t packet_id() { return 0x251; }
    static constexpr std::uint32_t default_priority() { return 4; }
    static std::optional<SegmentVoltageMessage> decode(util::Stream &stream, std::uint8_t sub_id);
    bool encode(util::Stream &stream) const;
};

struct SegmentTemperatureMessage : SegmentMessage {
    std::array<std::int8_t, 8> temperatures;

    static constexpr std::uint32_t packet_id() { return 0x252; }
    static constexpr std::uint32_t default_priority() { return 4; }
    static std::optional<SegmentTemperatureMessage> decode(util::Stream &stream, std::uint8_t sub_id);
    bool encode(util::Stream &stream) const;
};

struct StartFullDischargeMessage {
    std::uint16_t target_voltage;

    static constexpr std::uint32_t packet_id() { return 0x300; }
    static constexpr std::uint32_t default_priority() { return 5; }
    static std::optional<StartFullDischargeMessage> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const;
};

struct WriteConfigMessage {
    static constexpr std::uint32_t packet_id() { return 0x350; }
    static constexpr std::uint32_t default_priority() { return 7; }
    static std::optional<WriteConfigMessage> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const;
};

struct ConfigSegmentMessage {
    std::uint8_t start_address;
    std::uint8_t segment_count;
    std::uint8_t cell_count;
    std::uint8_t minimum_thermistor_count;

    static constexpr std::uint32_t packet_id() { return 0x351; }
    static constexpr std::uint32_t default_priority() { return 6; }
    static std::optional<ConfigSegmentMessage> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const;
};

struct ConfigThresholdMessage {
    std::uint16_t undervoltage_threshold;
    std::uint16_t overvoltage_threshold;
    std::uint16_t overcurrent_threshold;
    std::int8_t undertemperature_threshold;
    std::int8_t overtemperature_threshold;

    static constexpr std::uint32_t packet_id() { return 0x352; }
    static constexpr std::uint32_t default_priority() { return 6; }
    static std::optional<ConfigThresholdMessage> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const;
};

} // namespace bms
