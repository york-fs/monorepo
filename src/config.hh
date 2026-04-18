#pragma once

#include <can.hh>

#include <cstdint>

namespace config {

constexpr auto k_can_speed = can::Speed::_500;

constexpr std::uint8_t k_dti_can_id = 0x5;

constexpr std::uint8_t k_bms_can_id = 0x7;

constexpr std::uint8_t k_charger_can_id = 0x10;

// RPM to ERPM conversion factor. Emrax 228 has 10 motor pole pairs.
constexpr std::uint8_t k_erpm_factor = 10;

} // namespace config
