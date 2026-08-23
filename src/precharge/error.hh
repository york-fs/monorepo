#pragma once

#include <util/flag_bitset.hh>

#include <cstdint>

namespace precharge {

enum class Error : std::uint16_t {
    DischargeOpen,
    PrechargeClosed,
    AirPosClosed,
    AirNegClosed,
    PrecheckVoltage,
    WaitingDischarge,
    WaitingActivation,
    ShutdownOpen,
    PrechargeOpen,
    AirPosOpen,
    AirNegOpen,
    Deactivation,
    Deviation,
};

using ErrorFlags = util::FlagBitset<Error>;

} // namespace precharge
