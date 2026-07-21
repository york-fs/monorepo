#include <front/apps.hh>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace front {

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

} // namespace front
