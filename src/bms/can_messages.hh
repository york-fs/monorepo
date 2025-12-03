#pragma once

#include <bms/error.hh>
#include <util.hh>

#include <array>
#include <cstdint>

namespace bms {

// 100 Hz
class MasterStatusMessage {
    MasterErrorFlags m_error_flags;
    std::int8_t m_mcu_temperature;
    bool m_shutdown_activated;

public:
    bool encode(util::Stream &stream);

    bool is_shutdown_activated() const { return m_shutdown_activated; }
};

// 100 Hz
class MasterSummaryMessage {
    std::uint16_t m_estimated_soc;
    std::uint16_t m_min_voltage;
    std::uint16_t m_max_voltage;
    std::int8_t m_min_temperature;
    std::int8_t m_max_temperature;
};

// 100 Hz
class CurrentDataMessage {
    std::uint16_t m_positive_current;
    std::uint16_t m_negative_current;
};

// 100 Hz
class SegmentStatusMessage {
    SegmentErrorFlags m_error_flags;
    std::uint16_t m_cell_tap_bitset;
    std::uint16_t m_degraded_bitset;
};

// 5 Hz
class SegmentAuxiliaryMessage {
    std::uint16_t m_rail_voltage;
    std::int8_t m_mcu_temperature;
};

// 5 Hz
class VoltageDataMessage {
    std::array<std::uint16_t, 4> m_voltages;
};

// 5 Hz
class TemperatureDataMessage {
    std::array<std::int8_t, 8> m_temperatures;
};

} // namespace bms
