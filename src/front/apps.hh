#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>

namespace front {

/**
 * @brief The absolute start point in ADC counts. A measured value below this results in a sensor error.
 */
constexpr std::uint16_t k_absolute_start = 400;

/**
 * @brief The absolute end point in ADC counts. A measured value above this results in a sensor error.
 */
constexpr std::uint16_t k_absolute_end = 2000;

/**
 * @brief The computed size of the throttle map lookup table.
 */
constexpr std::size_t k_map_size = k_absolute_end - k_absolute_start;

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
