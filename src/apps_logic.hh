#pragma once

#include <array>
#include <cstdint>
#include <limits>

namespace apps {

// Total resolution of the ADC (12 bits).
constexpr std::uint16_t k_adc_range = 2u << 11u;

// Maximum tolerated delta between sensor ADC counts and the absolute endpoints [0, k_adc_range] before a sensor error
// is reported.
constexpr std::uint16_t k_absolute_delta = 1000;

// Maximum tolerated delta between sensor ADC counts and the calibrated endpoints before the system is deemed
// uncalibrated.
constexpr std::uint16_t k_relative_delta = 0;

// Minimum accepted total sensor range of the pedal in ADC counts.
constexpr std::uint16_t k_minimum_range = 300;

class ThrottleMap {
    // Store the worst case lookup table size, which is the full ADC range minus the null zones at the endpoints. Most
    // entries will end up being zero since in practice the sensor range is ~800 ADC counts.
    std::array<std::uint16_t, k_adc_range - k_absolute_delta * 2u> m_lookup_table{};
    
public:
    static ThrottleMap create_default();

    std::uint16_t operator[](std::size_t index) const { return m_lookup_table[index]; }
};

class Sensor {
    // TODO: Remove.
public:
    std::uint16_t m_start_value{std::numeric_limits<std::uint16_t>::max()};
    std::uint16_t m_min_value{std::numeric_limits<std::uint16_t>::max()};
    std::uint16_t m_max_value{0};
    bool m_calibrated{false};

    // Transient data used during calibration.
    std::array<std::uint16_t, 100> m_ring_buffer{};
    std::size_t m_ring_index{0};

public:
    std::uint16_t foo(std::uint16_t value);
    void update_calibration(std::uint16_t value);

    bool at_idle(std::uint16_t value) const;
    bool is_calibrated() const { return m_calibrated; }
};

} // namespace apps
