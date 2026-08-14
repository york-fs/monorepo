#pragma once

#include <util.hh>

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
