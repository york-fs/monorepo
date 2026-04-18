#pragma one

#include <util.hh>

#include <cstdint>
#include <optional>

namespace bms {

struct WriteConfigMessage {
    static constexpr std::uint32_t packet_id() { return 0x2000; }
    static constexpr std::uint32_t default_priority() { return 0; }
    static std::optional<WriteConfigMessage> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const;
};

struct ConfigSegmentMessage {
    std::uint8_t cell_count;
    std::uint8_t minimum_thermistor_count;

    static constexpr std::uint32_t packet_id() { return 0x2001; }
    static constexpr std::uint32_t default_priority() { return 1; }
    static std::optional<ConfigSegmentMessage> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const;
};

struct ConfigThresholdMessage {
    std::uint16_t undervoltage_threshold;
    std::uint16_t overvoltage_threshold;
    std::int8_t undertemperature_threshold;
    std::int8_t overtemperature_threshold;

    static constexpr std::uint32_t packet_id() { return 0x2002; }
    static constexpr std::uint32_t default_priority() { return 1; }
    static std::optional<ConfigThresholdMessage> decode(util::Stream &stream);
    bool encode(util::Stream &stream) const;
};

} // namespace bms
