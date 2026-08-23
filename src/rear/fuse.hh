#pragma once

#include <util/flag_bitset.hh>

#include <cstdint>

namespace rear {

enum class Fuse : std::uint32_t {
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
    RtdHorn = 10,
    Apps1 = 11,
    Apps2 = 12,
    Front = 13,
    Dwin = 14,
    Aux1 = 15,
    Aux2 = 16,
};

using FuseBitset = util::FlagBitset<Fuse>;

} // namespace rear
