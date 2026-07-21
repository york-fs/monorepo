#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace front {

constexpr std::uint16_t k_adc_range = 1u << 12;

constexpr std::uint16_t k_absolute_delta = 1000;

class ThrottleMap {
    std::array<std::uint16_t, k_adc_range - k_absolute_delta * 2> m_lut{};

public:
    static ThrottleMap create_default();

    std::uint16_t operator[](std::size_t index) const { return m_lut[index]; }
};

} // namespace front
