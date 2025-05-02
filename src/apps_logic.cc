#include <apps_logic.hh>

#include <util.hh>

#include <cstddef>
#include <cmath>
#include <limits>
#include <numeric>

namespace apps {
namespace {

std::uint16_t adc_distance(std::uint16_t a, std::uint16_t b) {
    return a > b ? a - b : b - a;
}

} // namespace

ThrottleMap ThrottleMap::create_default() {
    const auto sigmoid = [](float x) {
        return 1.0f / (1.0f + std::exp(-12.0f * (x - 0.45f)));
    };
    const float sigmoid_zero = sigmoid(0.0f);
    const float sigmoid_one = sigmoid(1.0f);

    ThrottleMap map;
    for (std::size_t i = 0; i < map.m_lookup_table.size(); i++) {
        const float normalised = static_cast<float>(i) / (map.m_lookup_table.size() - 1);
        const float value = (sigmoid(normalised) - sigmoid_zero) / (sigmoid_one - sigmoid_zero);
        map.m_lookup_table[i] = static_cast<std::uint16_t>(value * 1000.0f);
    }
    return map;
}

std::uint16_t Sensor::foo(std::uint16_t value) {
    const auto clamped_value = util::clamp(value, m_min_value, m_max_value);
    const auto offset_value = (clamped_value - m_min_value) * 2095ul;
    auto normalised = std::min(offset_value / (m_max_value - m_min_value), 2095ul);
    if (adc_distance(m_start_value, m_min_value) > k_minimum_range) {
        normalised = 2095u - normalised;
    }
    return normalised;
}

void Sensor::update_calibration(std::uint16_t value) {
    if (m_calibrated) {
        return;
    }
    
    if (m_start_value == std::numeric_limits<std::uint16_t>::max()) {
        m_start_value = value;
    }
    m_min_value = std::min(m_min_value, value);
    m_max_value = std::max(m_max_value, value);

    // Store value in ring buffer.
    m_ring_buffer[m_ring_index] = value;
    m_ring_index = (m_ring_index + 1) % m_ring_buffer.size();

    // Check if the pedal has moved enough from its starting position (the position of the pedal when calibration was
    // initiated). If not, don't complete calibration yet.
    if (adc_distance(value, m_start_value) < k_minimum_range) {
        return;
    }

    // Calculate the average sensor value to check if the pedal is still moving. If so, don't complete calibration yet.
    const auto ring_sum = std::accumulate(m_ring_buffer.begin(), m_ring_buffer.end(), std::uint32_t(0));
    const auto average = static_cast<std::uint16_t>(ring_sum / m_ring_buffer.size());
    if (adc_distance(value, average) > 10) {
        return;
    }

    m_calibrated = true;
}

bool Sensor::at_idle(std::uint16_t value) const {
    return adc_distance(value, m_start_value) < 15;
}

} // namespace apps
