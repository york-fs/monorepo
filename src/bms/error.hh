#pragma once

#include <util.hh>

#include <cstdint>

namespace bms {

enum class MasterError : std::uint32_t {
    BadCan = 0,
    BadConfig,
    BadEeprom,
    BadSensor,
    BadSegmentCount,
    DeadlineOverrun,

    BadCellCount,
    BadThermistorCount,
    Undervoltage,
    Overvoltage,
    Undertemperature,
    Overtemperature,
};

using MasterErrorFlags = util::FlagBitset<MasterError>;

enum class SegmentError : std::uint16_t {
    BadCellCount = 0,
    BadThermistorCount,
    Undervoltage,
    Overvoltage,
    Undertemperature,
    Overtemperature,
};

using SegmentErrorFlags = util::FlagBitset<SegmentError>;

} // namespace bms
