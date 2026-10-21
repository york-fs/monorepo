#include <bms/can_messages.hh>

#include <bms/error.hh>
#include <type_traits>
#include <util/stream.hh>
#include <util/type_traits.hh>

#include <array>
#include <concepts>
#include <cstdint>
#include <optional>

namespace bms {

std::optional<MasterStatusMessage> MasterStatusMessage::decode(util::Stream &stream) {
    const auto master_flags = stream.read_be<MasterErrorFlags::type_t>();
    const auto i2c_error_count = stream.read_be<std::uint32_t>();
    if (!master_flags || !i2c_error_count) {
        return std::nullopt;
    }
    return MasterStatusMessage{
        .master_flags = MasterErrorFlags(*master_flags),
        .i2c_error_count = *i2c_error_count,
    };
}

bool MasterStatusMessage::encode(util::Stream &stream) const {
    if (!stream.write_be(master_flags.value())) {
        return false;
    }
    return stream.write_be(i2c_error_count);
}

std::optional<MasterCurrentMessage> MasterCurrentMessage::decode(util::Stream &stream) {
    const auto positive_current = stream.read_be<std::int32_t>();
    const auto negative_current = stream.read_be<std::int32_t>();
    if (!positive_current || !negative_current) {
        return std::nullopt;
    }
    return MasterCurrentMessage{
        .positive_current = *positive_current,
        .negative_current = *negative_current,
    };
}

bool MasterCurrentMessage::encode(util::Stream &stream) const {
    if (!stream.write_be(positive_current)) {
        return false;
    }
    return stream.write_be(negative_current);
}

std::optional<MasterSummaryMessage> MasterSummaryMessage::decode(util::Stream &stream) {
    const auto min_voltage = stream.read_be<std::uint16_t>();
    const auto max_voltage = stream.read_be<std::uint16_t>();
    const auto min_temperature = stream.read_be<std::int8_t>();
    const auto max_temperature = stream.read_be<std::int8_t>();
    if (!min_voltage || !max_voltage || !min_temperature || !max_temperature) {
        return std::nullopt;
    }
    return MasterSummaryMessage{
        .min_voltage = *min_voltage,
        .max_voltage = *max_voltage,
        .min_temperature = *min_temperature,
        .max_temperature = *max_temperature,
    };
}

bool MasterSummaryMessage::encode(util::Stream &stream) const {
    if (!stream.write_be(min_voltage)) {
        return false;
    }
    if (!stream.write_be(max_voltage)) {
        return false;
    }
    if (!stream.write_be(min_temperature)) {
        return false;
    }
    return stream.write_be(max_temperature);
}

std::optional<ChargingStatusMessage> ChargingStatusMessage::decode(util::Stream &stream) {
    const auto state = stream.read_be<std::underlying_type_t<charging::State>>();
    if (!state) {
        return std::nullopt;
    }
    return ChargingStatusMessage{
        .state = charging::State(*state),
    };
}

bool ChargingStatusMessage::encode(util::Stream &stream) const {
    return stream.write_be(util::to_underlying(state));
}

static SegmentMessage decode_segment_sub_id(std::uint8_t sub_id) {
    return SegmentMessage{
        .segment_id = static_cast<std::uint8_t>((sub_id >> 2) & 0x3fu),
        .group_id = static_cast<std::uint8_t>(sub_id & 0b11u),
    };
}

std::optional<SegmentSummaryMessage> SegmentSummaryMessage::decode(util::Stream &stream, std::uint8_t sub_id) {
    const auto segment_flags = stream.read_be<SegmentErrorFlags::type_t>();
    const auto degraded_bitset = stream.read_be<std::uint16_t>();
    const auto balance_bitset = stream.read_be<std::uint16_t>();
    const auto max_correction = stream.read_be<std::uint16_t>();
    if (!segment_flags || !degraded_bitset || !balance_bitset || !max_correction) {
        return std::nullopt;
    }
    return SegmentSummaryMessage{
        decode_segment_sub_id(sub_id),
        SegmentErrorFlags(*segment_flags),
        *degraded_bitset,
        *balance_bitset,
        *max_correction,
    };
}

bool SegmentSummaryMessage::encode(util::Stream &stream) const {
    if (!stream.write_be(segment_flags.value())) {
        return false;
    }
    if (!stream.write_be(degraded_bitset)) {
        return false;
    }
    if (!stream.write_be(balance_bitset)) {
        return false;
    }
    return stream.write_be(max_correction);
}

std::optional<SegmentVoltageMessage> SegmentVoltageMessage::decode(util::Stream &stream, std::uint8_t sub_id) {
    std::array<std::uint16_t, 4> voltages;
    for (auto &out : voltages) {
        const auto voltage = stream.read_be<std::uint16_t>();
        if (!voltage) {
            return std::nullopt;
        }
        out = *voltage;
    }
    return SegmentVoltageMessage{
        decode_segment_sub_id(sub_id),
        voltages,
    };
}

bool SegmentVoltageMessage::encode(util::Stream &stream) const {
    for (const auto voltage : voltages) {
        if (!stream.write_be(voltage)) {
            return false;
        }
    }
    return true;
}

std::optional<SegmentTemperatureMessage> SegmentTemperatureMessage::decode(util::Stream &stream, std::uint8_t sub_id) {
    std::array<std::int8_t, 8> temperatures;
    for (auto &out : temperatures) {
        const auto temperature = stream.read_be<std::int8_t>();
        if (!temperature) {
            return std::nullopt;
        }
        out = *temperature;
    }
    return SegmentTemperatureMessage{
        decode_segment_sub_id(sub_id),
        temperatures,
    };
}

bool SegmentTemperatureMessage::encode(util::Stream &stream) const {
    for (const auto temperature : temperatures) {
        if (!stream.write_be(temperature)) {
            return false;
        }
    }
    return true;
}

std::optional<StartFullDischargeMessage> StartFullDischargeMessage::decode(util::Stream &stream) {
    const auto target_voltage = stream.read_be<std::uint16_t>();
    if (!target_voltage) {
        return std::nullopt;
    }
    return StartFullDischargeMessage{
        .target_voltage = *target_voltage,
    };
}

bool StartFullDischargeMessage::encode(util::Stream &stream) const {
    return stream.write_be(target_voltage);
}

std::optional<StartChargeMessage> StartChargeMessage::decode(util::Stream &) {
    return StartChargeMessage{};
}

bool StartChargeMessage::encode(util::Stream &) const {
    return true;
}

std::optional<WriteConfigMessage> WriteConfigMessage::decode(util::Stream &) {
    return WriteConfigMessage{};
}

bool WriteConfigMessage::encode(util::Stream &) const {
    return true;
}

std::optional<ConfigSegmentMessage> ConfigSegmentMessage::decode(util::Stream &stream) {
    const auto start_address = stream.read_byte();
    const auto segment_count = stream.read_byte();
    const auto cell_count = stream.read_byte();
    const auto minimum_thermistor_count = stream.read_byte();
    if (!start_address || !segment_count || !cell_count || !minimum_thermistor_count) {
        return std::nullopt;
    }
    return ConfigSegmentMessage{
        .start_address = *start_address,
        .segment_count = *segment_count,
        .cell_count = *cell_count,
        .minimum_thermistor_count = *minimum_thermistor_count,
    };
}

bool ConfigSegmentMessage::encode(util::Stream &stream) const {
    if (!stream.write_byte(start_address)) {
        return false;
    }
    if (!stream.write_byte(segment_count)) {
        return false;
    }
    if (!stream.write_byte(cell_count)) {
        return false;
    }
    return stream.write_byte(minimum_thermistor_count);
}

std::optional<ConfigThresholdMessage> ConfigThresholdMessage::decode(util::Stream &stream) {
    const auto undervoltage_threshold = stream.read_be<std::uint16_t>();
    const auto overvoltage_threshold = stream.read_be<std::uint16_t>();
    const auto overcurrent_threshold = stream.read_be<std::uint16_t>();
    const auto undertemperature_threshold = stream.read_byte();
    const auto overtemperature_threshold = stream.read_byte();
    if (!undervoltage_threshold || !overvoltage_threshold || !overcurrent_threshold || !undertemperature_threshold ||
        !overtemperature_threshold) {
        return std::nullopt;
    }
    return ConfigThresholdMessage{
        .undervoltage_threshold = *undervoltage_threshold,
        .overvoltage_threshold = *overvoltage_threshold,
        .overcurrent_threshold = *overcurrent_threshold,
        .undertemperature_threshold = static_cast<std::int8_t>(*undertemperature_threshold),
        .overtemperature_threshold = static_cast<std::int8_t>(*overtemperature_threshold),
    };
}

bool ConfigThresholdMessage::encode(util::Stream &stream) const {
    if (!stream.write_be(undervoltage_threshold)) {
        return false;
    }
    if (!stream.write_be(overvoltage_threshold)) {
        return false;
    }
    if (!stream.write_be(overcurrent_threshold)) {
        return false;
    }
    if (!stream.write_byte(undertemperature_threshold)) {
        return false;
    }
    return stream.write_byte(overtemperature_threshold);
}

} // namespace bms
