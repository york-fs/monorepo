#pragma once

#include <array>
#include <concepts>
#include <cstdint>
#include <span>
#include <variant>

namespace can {

/// An enum which represents which GPIO port is being used for CAN.
enum class Port {
    /// RX on PA11 and TX on PA12.
    A,

    /// RX on PB8 and TX on PB9.
    B,

    /// RX on PD0 and TX on PD1.
    D,
};

/// An enum which represents the different available bus speeds.
enum class Speed {
    /// 33.3333 kbit/s.
    _33_3,

    /// 500 kbit/s.
    _500,

    /// 1000 kbit/s.
    _1000,
};

/// A class to make distinct numerical types for CAN identifiers.
template <int N, std::integral T>
struct BaseIdentifier {
    using Self = BaseIdentifier<N, T>;
    T value;

    BaseIdentifier(T value) : value(value) {}
    constexpr operator T() const { return value; }
};

/// 11-bit standard identifier.
using StandardIdentifier = BaseIdentifier<0, std::uint16_t>;

/// 29-bit extended identifier.
using ExtendedIdentifier = BaseIdentifier<1, std::uint32_t>;

/// CAN message identifier.
using Identifier = std::variant<StandardIdentifier, ExtendedIdentifier>;

/// A struct which represents a CAN message.
struct Message {
    Identifier identifier;
    std::array<std::uint8_t, 8> data;
    std::uint8_t length;

    bool operator==(const Message &) const = default;

    /**
     * @return the standard identifier of the message
     */
    StandardIdentifier standard_id() const { return std::get<StandardIdentifier>(identifier); }

    /**
     * @return the extended identifier of the message
     */
    ExtendedIdentifier extended_id() const { return std::get<ExtendedIdentifier>(identifier); }

    /**
     * @return true if the message has a standard identifier; false otherwise
     */
    bool is_standard() const { return std::holds_alternative<StandardIdentifier>(identifier); }

    /**
     * @return true if the message has an extended identifier; false otherwise
     */
    bool is_extended() const { return std::holds_alternative<ExtendedIdentifier>(identifier); }
};

/// CAN FIFO callback function type.
using fifo_callback_t = void (*)(const Message &);

/**
 * Initialises the CAN1 peripheral to 500 kbits/s. Assumes a 28 MHz APB1 clock.
 *
 * @param port the pin pair to use as RX and TX
 * @param speed the bus speed to use
 * @return true if initialisation was successful; false otherwise
 */
[[nodiscard]] bool init(Port port, Speed speed);

/**
 * Configures and enables the specified CAN filter to route incoming messages to the specified FIFO index. A
 * message will be matched if, after applying the given bitmask, it equals the specified value.
 *
 * Incoming extended messages are structured as follows:
 *   EXID[28:0] | IDE | RTR | 0
 *
 * Incoming standard messages are structured as follows:
 *   STID[10:0] | 0[17:0] | IDE | RTR | 0
 *
 * @param filter the filter index to configure; must be in the range [0, 13]
 * @param fifo the FIFO index; must be 0 or 1
 * @param mask the bitmask to be applied to the incoming message (bitwise AND)
 * @param value the expected value to which the incoming message is compared to, after masking
 */
void route_filter(std::uint8_t filter, std::uint8_t fifo, std::uint32_t mask, std::uint32_t value);

/**
 * Sets the given callback to be called from the message pending interrupt of the specified FIFO index.
 */
void set_fifo_callback(std::uint8_t index, fifo_callback_t callback);

/**
 * Queues the given message for transmission on the CAN bus.
 *
 * @return true if the message was successfully placed into a FIFO; false otherwise
 */
bool transmit(const Message &message);

/**
 * Builds a CAN message given an identifier and an array of bytes. The number of bytes will be truncated to eight
 * bytes maximum.
 */
inline Message build_raw(Identifier identifier, std::span<const std::uint8_t> data) {
    Message message{
        .identifier = identifier,
        .length = static_cast<std::uint8_t>(data.size() > 8 ? 8 : data.size()),
    };
    for (std::uint8_t i = 0; i < message.length; i++) {
        message.data[i] = data[i];
    }
    return message;
}

/**
 * Builds a CAN message given a standard identifier and an array of bytes. The number of bytes will be truncated to
 * eight bytes maximum.
 */
inline Message build_standard(StandardIdentifier identifier, std::span<const std::uint8_t> data) {
    return build_raw(identifier, data);
}

/**
 * Builds a CAN message given an extended identifier and an array of bytes. The number of bytes will be truncated to
 * eight bytes maximum.
 */
inline Message build_extended(ExtendedIdentifier identifier, std::span<const std::uint8_t> data) {
    return build_raw(identifier, data);
}

} // namespace can
