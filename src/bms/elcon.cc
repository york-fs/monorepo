#include <bms/elcon.hh>

#include <util/stream.hh>

#include <cstdint>
#include <optional>

namespace elcon {

std::optional<StatusMessage> StatusMessage::decode(util::Stream &stream, std::uint8_t) {
    const auto output_voltage = stream.read_be<std::uint16_t>();
    const auto output_current = stream.read_be<std::uint16_t>();
    const auto status_flags = stream.read_be<StatusFlags::type_t>();
    if (!output_voltage || !output_current || !status_flags) {
        return std::nullopt;
    }
    return StatusMessage{
        .output_voltage = *output_voltage,
        .output_current = *output_current,
        .status_flags = StatusFlags(*status_flags),
    };
}

bool ControlMessage::encode(util::Stream &stream) const {
    if (!stream.write_be(target_voltage)) {
        return false;
    }
    if (!stream.write_be(target_current)) {
        return false;
    }
    if (!stream.write_byte(disable ? 1 : 0)) {
        return false;
    }

    // Padding bytes.
    return stream.write_byte(0) && stream.write_byte(0) && stream.write_byte(0);
}

} // namespace elcon
