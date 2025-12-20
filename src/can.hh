#pragma once

#include <array>
#include <concepts>
#include <cstdint>
#include <optional>
#include <span>
#include <variant>

namespace can {

/**
 * @brief The GPIO ports which can be used for the CAN lines.
 */
enum class Port {
    /**
     * @brief RX on PA11 and TX on PA12.
     */
    A,

    /**
     * @brief PB8 and TX on PB9.
     */
    B,

    /**
     * @brief RX on PD0 and TX on PD1.
     */
    D,
};

/**
 * @brief The available bus speeds the peripheral can be configured to.
 */
enum class Speed {
    /**
     * @brief 33.3333 kbit/s.
     */
    _33_3,

    /**
     * @brief 500 kbit/s.
     */
    _500,

    /**
     * 1000 kbit/s.
     */
    _1000,
};

/**
 * @brief A class to make distinct numerical types for CAN identifiers.
 */
template <int N, std::integral T>
struct BaseIdentifier {
    using Self = BaseIdentifier<N, T>;
    T value;

    BaseIdentifier(T value) : value(value) {}
    constexpr operator T() const { return value; }
};

/**
 * @brief An 11-bit CAN 2.0A standard identifier.
 */
using StandardIdentifier = BaseIdentifier<0, std::uint16_t>;

/**
 * @brief A 29-bit CAN 2.0B extended identifier.
 */
using ExtendedIdentifier = BaseIdentifier<1, std::uint32_t>;

/**
 * @brief CAN frame identifier variant.
 */
using Identifier = std::variant<StandardIdentifier, ExtendedIdentifier>;

/**
 * @brief A struct which represents an encoded CAN frame.
 */
struct Frame {
    Identifier identifier;
    std::array<std::uint8_t, 8> data;
    std::uint8_t length;

    bool operator==(const Frame &) const = default;

    /**
     * @return the standard identifier of the frame
     */
    StandardIdentifier standard_id() const { return std::get<StandardIdentifier>(identifier); }

    /**
     * @return the extended identifier of the frame
     */
    ExtendedIdentifier extended_id() const { return std::get<ExtendedIdentifier>(identifier); }

    /**
     * @return true if the frame has a standard identifier; false otherwise
     */
    bool is_standard() const { return std::holds_alternative<StandardIdentifier>(identifier); }

    /**
     * @return true if the frame has an extended identifier; false otherwise
     */
    bool is_extended() const { return std::holds_alternative<ExtendedIdentifier>(identifier); }
};

/**
 * @brief CAN FIFO callback function type.
 */
using fifo_callback_t = void (*)(const Frame &);

/**
 * @brief Initialises the CAN1 peripheral to the given bus speed. Assumes a 28 MHz APB1 clock.
 *
 * @param port the pin pair to use as RX and TX
 * @param speed the bus speed to use
 * @return true if initialisation was successful; false otherwise
 */
[[nodiscard]] bool init(Port port, Speed speed);

/**
 * @brief Configures and enables the specified CAN filter to route incoming frames to the specified FIFO index. A
 * frame will be matched if, after applying the given bitmask, it equals the specified value.
 *
 * Incoming extended frames are structured as follows:
 *   EXID[28:0] | IDE | RTR | 0
 *
 * Incoming standard frames are structured as follows:
 *   STID[10:0] | 0[17:0] | IDE | RTR | 0
 *
 * @param filter the filter index to configure; must be in the range [0, 13]
 * @param fifo the FIFO index; must be 0 or 1
 * @param mask the bitmask to be applied to the incoming frame (bitwise AND)
 * @param value the expected value to which the incoming frame is compared to, after masking
 */
void route_filter(std::uint8_t filter, std::uint8_t fifo, std::uint32_t mask, std::uint32_t value);

/**
 * @brief Sets the given callback to be called from the message pending interrupt of the specified FIFO index.
 */
void set_fifo_callback(std::uint8_t index, fifo_callback_t callback);

/**
 * @brief Queues the given frame for transmission on the CAN bus.
 *
 * @return true if the frame was successfully placed into a free FIFO; false otherwise
 */
bool transmit(const Frame &frame);

/**
 * @brief Builds a CAN frame from the given identifier and an array of data bytes. The number of bytes will be truncated
 * to eight bytes maximum.
 */
inline Frame build_raw(Identifier identifier, std::span<const std::uint8_t> data) {
    Frame frame{
        .identifier = identifier,
        .length = static_cast<std::uint8_t>(data.size() > 8 ? 8 : data.size()),
    };
    for (std::uint8_t i = 0; i < frame.length; i++) {
        frame.data[i] = data[i];
    }
    return frame;
}

/**
 * @brief Builds a CAN frame from the given standard identifier and an array of data bytes. The number of bytes will be
 * truncated to eight bytes maximum.
 */
inline Frame build_standard(StandardIdentifier identifier, std::span<const std::uint8_t> data) {
    return build_raw(identifier, data);
}

/**
 * @brief Builds a CAN frame from the given extended identifier and an array of data bytes. The number of bytes will be
 * truncated to eight bytes maximum.
 */
inline Frame build_extended(ExtendedIdentifier identifier, std::span<const std::uint8_t> data) {
    return build_raw(identifier, data);
}

} // namespace can
