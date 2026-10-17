#pragma once

#include <hal/can.hh>

#include <cstdint>

namespace config {

constexpr auto k_can_speed = hal::can::Speed::_500;

constexpr std::uint8_t k_rear_can_id = 0x1;
constexpr std::uint8_t k_front_can_id = 0x2;
constexpr std::uint8_t k_bms_can_id = 0x3;
constexpr std::uint8_t k_precharge_can_id = 0x4;
constexpr std::uint8_t k_dti_can_id = 0x5;
constexpr std::uint8_t k_charger_can_id = 0x6;

/**
 * @brief Globally consistent period for status message sending rate in milliseconds.
 *
 * Status messages are generally the messages whose timing is critical and is checking on the receiving end via
 * TimeTracked objects, as opposed to other messages whose data is used for more general telemetry gathering rather than
 * important decision-making logic.
 */
constexpr std::uint32_t k_status_period = 20;

/**
 * @brief Throttle sensing and sending period in milliseconds.
 */
constexpr std::uint32_t k_throttle_period = 10;

/**
 * @brief Helper function for calculating a suitable timeout value for TimeTracked CAN messages.
 */
consteval std::uint32_t can_timeout(std::uint32_t period) {
    // Return a 2.5x value. This allows for at least one dropped CAN frame to be tolerated.
    return period * 5 / 2;
}

consteval bool enable_debug_logs() {
#ifdef NDEBUG
    return false;
#else
    return true;
#endif
}

} // namespace config
