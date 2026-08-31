#pragma once

#include <can.hh>

#include <cstdint>

namespace config {

constexpr auto k_can_speed = can::Speed::_500;

constexpr std::uint8_t k_rear_can_id = 0x1;
constexpr std::uint8_t k_front_can_id = 0x2;
constexpr std::uint8_t k_bms_can_id = 0x3;
constexpr std::uint8_t k_precharge_can_id = 0x4;
constexpr std::uint8_t k_dti_can_id = 0x5;
constexpr std::uint8_t k_charger_can_id = 0x6;

consteval bool enable_debug_logs() {
#ifdef NDEBUG
    return false;
#else
    return true;
#endif
}

} // namespace config
