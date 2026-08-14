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

inline bool is_state_active(State state) {
    switch (state) {
    case State::Precharge:
    case State::PrechargeHold:
    case State::Active:
        return true;
    default:
        return false;
    }
}

} // namespace precharge
