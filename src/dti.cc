#include <dti.hh>

#include <can.hh>
#include <util.hh>

#include <cstdint>

namespace dti {
namespace {

// Messages from the inverter.
constexpr std::uint32_t k_general_data_1_id = 0x20;
constexpr std::uint32_t k_general_data_2_id = 0x21;
constexpr std::uint32_t k_general_data_3_id = 0x22;
constexpr std::uint32_t k_general_data_5_id = 0x24;


} // namespace

Packet parse_packet(const can::Message &message) {
    // Extract packet ID (upper 21 bits) from extended CAN ID.
    const auto packet_id = (message.identifier >> 8u) & 0x1fffffu;
    std::span<const std::uint8_t> span = message.data;

    // TODO: Should probably check message.length.
    switch (packet_id) {
    case k_general_data_1_id:
        return GeneralData1{
            .erpm = util::read_be<std::int32_t>(span.subspan<0, 4>()),
            .duty_cycle = util::read_be<std::int16_t>(span.subspan<4, 2>()),
            .input_voltage = util::read_be<std::int16_t>(span.subspan<6, 2>()),
        };
    case k_general_data_2_id:
        return GeneralData2{
            .ac_current = util::read_be<std::int16_t>(span.subspan<0, 2>()),
            .dc_current = util::read_be<std::int16_t>(span.subspan<2, 2>()),
        };
    case k_general_data_3_id:
        return GeneralData3{
            .controller_temperature = util::read_be<std::int16_t>(span.subspan<0, 2>()),
            .motor_temperature = util::read_be<std::int16_t>(span.subspan<2, 2>()),
            .fault_code = static_cast<FaultCode>(span[5]),
        };
    case k_general_data_5_id:
        return GeneralData5{
            .throttle = static_cast<std::int8_t>(span[0]),
            .brake = static_cast<std::int8_t>(span[1]),
            .digital_pin_state = span[2],
            .drive_enabled = span[3] != 0u,
            .capacitor_temperature_limit_active = ((span[4] >> 0u) & 1u) != 0u,
            .dc_current_limit_active = ((span[4] >> 1u) & 1u) != 0u,
            .drive_enable_limit_active = ((span[4] >> 2u) & 1u) != 0u,
            .igbt_acceleration_limit_active = ((span[4] >> 3u) & 1u) != 0u,
            .igbt_temperature_limit_active = ((span[4] >> 4u) & 1u) != 0u,
            .input_voltage_limit_active = ((span[4] >> 5u) & 1u) != 0u,
            .motor_acceleration_temperature_limit_active = ((span[4] >> 6u) & 1u) != 0u,
            .motor_temperature_limit_active = ((span[4] >> 7u) & 1u) != 0u,
            .rpm_min_limit_active = ((span[5] >> 0u) & 1u) != 0u,
            .rpm_max_limit_active = ((span[5] >> 1u) & 1u) != 0u,
            .power_limit_active = ((span[5] >> 2u) & 1u) != 0u,
            .can_map_version = span[7],
        };
    }

    return UnknownMessageType{
        .packet_id = packet_id,
    };
}

} // namespace dti
