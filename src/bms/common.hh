#pragma once

#include <cstdint>

namespace bms {

/**
 * @brief Maximum connected cell count.
 */
constexpr std::uint32_t k_cell_count = 16;

/**
 * @brief Maximum connected thermistor count, including the on-board ones.
 */
constexpr std::uint32_t k_thermistor_count = 24;

} // namespace bms
