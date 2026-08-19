#pragma once

#include <util.hh>

#include <cstdint>

namespace rear {

enum class ShutdownCircuitOpenCause : std::uint8_t {
    None,
    RearInput,
    FrontEstop,
    BrakeOverTravel,
    InertiaSwitch,
    FrontAuxiliary,
    FrontOutput,
    BmsLatch,
    ImdLatch,
    InverterInterlock,
    ShutdownLatchFailure,
    LeftEstop,
    RightEstop,
    HvdInterlock,
    RearAuxiliary,
    Tsms,
};

enum class TsPreventionFlag : std::uint8_t {
    ShutdownOpen,
    BadFuse,

    // Front.
    FrontOffline,
    NotRequested,

    // Precharge.
    PrechargeOffline,
    PrechargeState,
};

using TsPreventionFlags = util::FlagBitset<TsPreventionFlag>;

enum class RtdPreventionFlag : std::uint8_t {
    TsNotActive,
    NotRequested,
    BrakeNotPressed,
};

using RtdPreventionFlags = util::FlagBitset<RtdPreventionFlag>;

} // namespace rear
