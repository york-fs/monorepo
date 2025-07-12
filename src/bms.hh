#pragma once

#include <util.hh>

#include <array>
#include <cstdint>

namespace bms {

enum class Error : std::uint32_t {
    BadCan = 0,
    BadConfig,
    BadEeprom,
    BadSensor,
    SegmentCount,
};

using ErrorFlags = util::FlagBitset<Error>;

struct SegmentData {
    // For a given thermistor index, the bit corresponding to that index is:
    //   0 if the thermistor is disconnected or otherwise reading out of range;
    //   1 if the thermistor is connected and reading properly.
    std::uint32_t thermistor_bitset;

    // For a given cell index, the bit corresponding to that index is:
    //   0 if the cell tap is disconnected;
    //   1 if the cell tap is connected.
    std::uint16_t cell_tap_bitset;

    // For a given cell index, the bit corresponding to that index is:
    //   0 if the cell voltage reading is as expected;
    //   1 if the reading is noisy or otherwise less reliable than normal.
    std::uint16_t degraded_bitset;

    // 3V3 rail voltage in 100 uV resolution.
    std::uint16_t rail_voltage;

    // Cell voltages in 100 uV resolution.
    std::array<std::uint16_t, 12> voltages;

    // Thermistor temperatures to the nearest degree.
    std::array<std::int8_t, 23> temperatures;

    // True if the segment is ready and the data is valid.
    bool valid;
};

} // namespace bms
