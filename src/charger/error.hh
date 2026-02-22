#pragma once

#include <util.hh>

#include <cstdint>

namespace charger {

enum class Error : std::uint16_t {
    /**
     * @brief Disabled by CAN command.
     */
    Disabled,
    CanOffline,
    RailUndervoltage,
    RailOvervoltage,
    DropOvervoltage,
    Overcurrent,
    Overtemperature,
    CommunicationTimeout,
};

using ErrorFlags = util::FlagBitset<Error>;

} // namespace charger
