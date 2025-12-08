#pragma once

#include <util.hh>

#include <cstdint>

namespace bms {

enum class MasterError : std::uint32_t {
    BadCan,
    BadConfig,
    BadEeprom,
    BadSegmentCount,
    DeadlineOverrun,
    SegmentError,
};

using MasterErrorFlags = util::FlagBitset<MasterError>;

enum class SegmentError : std::uint16_t {
    Disconnected,
    BadCellCount,
    BadThermistorCount,
    BadRailVoltage,
    UnreliableCommunication,
    UnreliableMeasurement,
    Undervoltage,
    Overvoltage,
    Undertemperature,
    Overtemperature,
};

using SegmentErrorFlags = util::FlagBitset<SegmentError>;

} // namespace bms
