#include <bms.hh>

#include <bit>
#include <cstdint>
#include <limits>
#include <utility>

namespace bms {

std::pair<std::uint16_t, std::uint16_t> min_max_voltage(const SegmentData &data) {
    std::uint16_t min_voltage = std::numeric_limits<std::uint16_t>::max();
    std::uint16_t max_voltage = 0;
    for (std::size_t i = 0; i < data.voltages.size(); i++) {
        if ((data.cell_tap_bitset & (1u << i)) != 0u) {
            min_voltage = std::min(min_voltage, data.voltages[i]);
            max_voltage = std::max(max_voltage, data.voltages[i]);
        }
    }
    return std::make_pair(min_voltage, max_voltage);
}

std::pair<std::int8_t, std::int8_t> min_max_temperature(const SegmentData &data) {
    std::int8_t min_temperature = std::numeric_limits<std::int8_t>::max();
    std::int8_t max_temperature = std::numeric_limits<std::int8_t>::min();
    for (std::size_t i = 0; i < data.temperatures.size(); i++) {
        if ((data.thermistor_bitset & (1u << i)) != 0u) {
            min_temperature = std::min(min_temperature, data.temperatures[i]);
            max_temperature = std::max(max_temperature, data.temperatures[i]);
        }
    }
    return std::make_pair(min_temperature, max_temperature);
}

ErrorFlags check_segment(const Config &config, const SegmentData &data) {
    ErrorFlags flags;
    if (std::popcount(data.cell_tap_bitset) != config.expected_cell_count) {
        // One or more cell taps disconnected.
        flags.set(Error::CellCount);
    }
    if (std::popcount(data.thermistor_bitset) < config.minimum_thermistor_count) {
        // Below minimum amount of required thermistors.
        flags.set(Error::ThermistorCount);
    }

    // Check cell voltages.
    const auto [min_voltage, max_voltage] = min_max_voltage(data);
    if (min_voltage < config.minimum_cell_voltage) {
        flags.set(Error::Undervoltage);
    }
    if (max_voltage > config.maximum_cell_voltage) {
        flags.set(Error::Overvoltage);
    }

    // Check temperatures.
    const auto [min_temperature, max_temperature] = min_max_temperature(data);
    if (min_temperature < config.minimum_temperature) {
        flags.set(Error::Undertemperature);
    }
    if (max_temperature > config.maximum_temperature) {
        flags.set(Error::Overtemperature);
    }
    return flags;
}

} // namespace bms
