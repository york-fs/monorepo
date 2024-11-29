#include "dti.hh"

#include <cstdint>

namespace dti {
namespace {

constexpr std::uint32_t k_general_data_1_id = 0x2a;
constexpr std::uint32_t k_general_data_2_id = 0x2b;
constexpr std::uint32_t k_general_data_3_id = 0x2c;
constexpr std::uint32_t k_general_data_5_id = 0x2e;

template <typename T>
T read_be(std::uint32_t data_word) {
    if constexpr (sizeof(T) == sizeof(std::uint8_t)) {
        return static_cast<T>(data_word);
    }
    if constexpr (sizeof(T) == sizeof(std::uint16_t)) {
        return static_cast<T>(__builtin_bswap16(data_word));
    }
    if constexpr (sizeof(T) == sizeof(std::uint32_t)) {
        return static_cast<T>(__builtin_bswap32(data_word));
    }
}

} // namespace

Packet parse_packet(const std::uint32_t ext_id, const std::uint32_t data_low, const std::uint32_t data_high) {
    // Extract packet ID (upper 21 bits) from extended CAN ID.
    const auto packet_id = (ext_id >> 8u) & 0x1fffffu;
    switch (packet_id) {
    case k_general_data_1_id:
        return GeneralData1 {
            .erpm = read_be<std::int32_t>(data_low),
            .duty_cycle = read_be<std::int16_t>(data_high & 0xffffu),
            .input_voltage = read_be<std::int16_t>(data_high >> 16u),
        };
    case k_general_data_2_id:
        return GeneralData2{
            .ac_current = read_be<std::int16_t>(data_low & 0xffffu),
            .dc_current = read_be<std::int16_t>(data_low >> 16u),
        };
    case k_general_data_3_id:
        return GeneralData3{
            .controller_temperature = read_be<std::int16_t>(data_low & 0xffffu),
            .motor_temperature = read_be<std::int16_t>(data_low >> 16u),
            .fault_code = read_be<FaultCode>(data_high & 0xffu),
        };
    case k_general_data_5_id:
        return GeneralData5{
            .throttle = read_be<std::int8_t>(data_low & 0xffu),
            .brake = read_be<std::int8_t>((data_low >> 8u) & 0xffu),
            .digital_pin_state = read_be<std::uint8_t>((data_low >> 16u) & 0xffu),
            .drive_enable = ((data_low >> 24u) & 1u) != 0u,
            .capacitor_temperature_limit_active = ((data_high >> 0u) & 1u) != 0u,
            .dc_current_limit_active = ((data_high >> 1u) & 1u) != 0u,
            .drive_enable_limit_active = ((data_high >> 2u) & 1u) != 0u,
            .igbt_acceleration_limit_active = ((data_high >> 3u) & 1u) != 0u,
            .igbt_temperature_limit_active = ((data_high >> 4u) & 1u) != 0u,
            .input_voltage_limit_active = ((data_high >> 5u) & 1u) != 0u,
            .motor_acceleration_temperature_limit_active = ((data_high >> 6u) & 1u) != 0u,
            .motor_temperature_limit_active = ((data_high >> 7u) & 1u) != 0u,
            .rpm_min_limit_active = ((data_high >> 8u) & 1u) != 0u,
            .rpm_max_limit_active = ((data_high >> 9u) & 1u) != 0u,
            .power_limit_active = ((data_high >> 10u) & 1u) != 0u,
        };
    default:
        return UnknownData{
            .data_low = data_low,
            .data_high = data_high,
        };
    }
}

} // namespace dti
