#pragma once

#include <cstdint>

namespace precharge {

enum class State : std::uint8_t {
    LedCheck,
    Precheck,
    Standby,
    Precharge,
    PrechargeHold,
    Active,
};

} // namespace precharge
