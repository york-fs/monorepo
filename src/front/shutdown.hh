#pragma once

#include <util/flag_bitset.hh>

#include <cstdint>

namespace front {

enum class ShutdownSample : std::uint8_t {
    EmergencyStop,
    BrakeOverTravel,
    InertiaSwitch,
    Auxiliary,
};

using ShutdownSamples = util::FlagBitset<ShutdownSample>;

} // namespace front
