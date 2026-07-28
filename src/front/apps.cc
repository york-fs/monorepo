#include <front/apps.hh>
#include <util.hh>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numeric>

namespace front {
namespace {

std::uint16_t abs_distance(std::uint16_t lhs, std::uint16_t rhs) {
    return lhs > rhs ? lhs - rhs : rhs - lhs;
}

} // namespace

ThrottleMap ThrottleMap::create_default() {
    const auto sigmoid = [](float x) {
        return 1.0f / (1.0f + std::exp(-12.0f * (x - 0.45f)));
    };
    const auto sigmoid_zero = sigmoid(0.0f);
    const auto sigmoid_one = sigmoid(1.0f);

    ThrottleMap map;
    for (std::size_t i = 0; i < map.m_lut.size(); i++) {
        const auto normalised = static_cast<float>(i) / (map.m_lut.size() - 1);
        const auto value = (sigmoid(normalised) - sigmoid_zero) / (sigmoid_one - sigmoid_zero);
        map.m_lut[i] = static_cast<std::uint16_t>(value * 1000.0f + 0.99f);
    }
    return map;
}

std::optional<std::uint16_t> Sensor::normalise(std::uint16_t value) const {
    // Reject if outside of absolute allowed range.
    if (value < k_absolute_delta || value > (k_adc_range - k_absolute_delta)) {
        return std::nullopt;
    }

    // Clamp value to calibrated limits.
    value = util::clamp(value, m_min_value, m_max_value);

    // Normalise value between calibrated limits.
    constexpr auto range = k_adc_range - k_absolute_delta * 2 - 1;
    return ((value - m_min_value) * range) / (m_max_value - m_min_value);
}

void Sensor::update_limits(std::uint16_t value) {
    m_min_value = std::min(m_min_value, value);
    m_max_value = std::max(m_max_value, value);
}

bool Calibrator::update(std::uint16_t value) {
    // Update running ranges and average.
    m_start_value = m_start_value.value_or(value);
    m_ring_buffer[m_ring_index] = value;
    m_ring_index = (m_ring_index + 1) % m_ring_buffer.size();

    // Check if the pedal has moved enough from its start position.
    if (abs_distance(value, *m_start_value) < 100) {
        return false;
    }

    const auto ring_sum = std::accumulate(m_ring_buffer.begin(), m_ring_buffer.end(), std::uint32_t(0));
    const auto average = static_cast<std::uint16_t>(ring_sum / m_ring_buffer.size());
    return abs_distance(value, average) < 10;
}

} // namespace front
