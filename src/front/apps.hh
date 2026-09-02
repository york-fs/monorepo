#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>

namespace front {

/**
 * @brief Total ADC resolution range.
 */
constexpr std::uint16_t k_adc_range = 1u << 12;

/**
 * @brief The maximum tolerated delta between sensor ADC counts and the absolute endpoints [0, k_adc_range] before a
 * sensor error is reported.
 */
constexpr std::uint16_t k_absolute_delta = 1000;

constexpr std::size_t k_map_size = k_adc_range - k_absolute_delta * 2;

class ThrottleMap {
    std::array<std::uint16_t, k_map_size> m_lut{};

public:
    static ThrottleMap create_default();
    static std::uint16_t to_percentage(std::uint16_t normalised);

    std::uint16_t operator()(std::size_t index) const { return m_lut[index]; }
};

class Sensor {
    std::uint16_t m_min_value{std::numeric_limits<std::uint16_t>::max()};
    std::uint16_t m_max_value{std::numeric_limits<std::uint16_t>::min()};

public:
    std::optional<std::uint16_t> normalise(std::uint16_t value) const;
    void update_limits(std::uint16_t value);
};

class Calibrator {
    std::array<std::uint16_t, 100> m_ring_buffer;
    std::uint16_t m_ring_index{0};
    std::optional<std::uint16_t> m_start_value;

public:
    bool update(std::uint16_t value);
};

} // namespace front
