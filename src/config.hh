#pragma once

#include <cstdint>

namespace config {

constexpr std::uint8_t k_dti_can_id = 0x5;

// RPM to ERPM conversion factor. Emrax 228 has 10 motor pole pairs.
constexpr std::uint8_t k_erpm_factor = 10;

} // namespace config
