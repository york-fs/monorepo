#pragma once

#include <util.hh>

#include <cstdint>

namespace rear {

enum class Fuse : std::uint16_t {
    Bms = 0,
    Imd = 1,
    TsacFans = 2,
    Precharge = 3,
    CoolantPump = 4,
    BrakeLight = 5,
    TsalLed = 6,
    Inverter = 7,
    ShutdownLatch = 8,
    EnergyMeter = 9,
};

using FuseBitset = util::FlagBitset<Fuse>;

} // namespace rear
