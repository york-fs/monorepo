#pragma once

#include <util.hh>

#include <cstdint>

namespace precharge {

enum class RelayState : std::uint8_t {
    DischargeClosed,
    PrechargeClosed,
    AirPosClosed,
    AirNegClosed,
};

using RelayStates = util::FlagBitset<RelayState>;

} // namespace precharge
