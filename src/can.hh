#pragma once

#include <util.hh>

#include <array>
#include <cassert>
#include <concepts>
#include <cstdint>
#include <optional>
#include <span>
#include <variant>

namespace can {

template <typename T>
concept HasId = requires {
    { T::packet_id() } -> std::same_as<std::uint32_t>;
} && T::packet_id() < 0x3ffff;

template <typename T>
concept Decodable = HasId<T> && requires(util::Stream &stream) {
    { T::decode(stream) } -> std::same_as<std::optional<T>>;
};

template <typename T>
concept Encodable = HasId<T> && requires(T message, util::Stream &stream) {
    { message.encode(stream) } -> std::same_as<bool>;
};

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
using Identifier = std::variant<std::monostate, StandardIdentifier, ExtendedIdentifier>;

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

struct Stats {
    std::uint32_t rx_count;
    std::uint32_t tx_count;
    std::uint32_t lost_rx_count;
    std::uint32_t lost_tx_count;
};

/**
 * @brief CAN received frame callback function type.
 */
using rx_callback_t = void (*)(const Frame &);

/**
 * @brief Initialises the CAN1 peripheral to the given bus speed and starts the transmission task. Assumes a 28 MHz APB1
 * clock.
 *
 * @param port the pin pair to use as RX and TX
 * @param speed the bus speed to use
 * @param task_priority the priority to use for the transmission task
 * @return true if initialisation was successful; false otherwise
 */
void init(Port port, Speed speed, std::uint32_t task_priority);

/**
 * @brief Sets the given callback to be called when a message is pending on the given FIFO and filter index.
 *
 * @param fifo the FIFO index; must be 0 or 1
 * @param filter the filter index; must be in the range [0, 13]
 * @param callback the callback to set
 */
void set_rx_callback(std::uint8_t fifo, std::uint8_t filter, rx_callback_t callback);

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
 * @param fifo the FIFO index; must be 0 or 1
 * @param filter the filter index to configure; must be in the range [0, 13]
 * @param mask the bitmask to be applied to the incoming frame (bitwise AND)
 * @param value the expected value to which the incoming frame is compared to, after masking
 */
void route_filter(std::uint8_t fifo, std::uint8_t filter, std::uint32_t mask, std::uint32_t value);

/**
 * @brief Queues the given frame for transmission on the CAN bus.
 */
void queue_frame(const Frame &frame);

/**
 * @return true if the CAN peripheral is online and not in a bus off state.
 */
bool is_online();

/**
 * @return reception and transmission stats since boot.
 */
Stats get_stats();

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

/**
 * @brief Decodes a without checking node or packet IDs.
 *
 * @param frame the frame to decode
 * @return a T if successful; otherwise std::nullopt if the stream decoding failed
 */
template <Decodable T>
std::optional<T> decode_fast(const Frame &frame) {
    util::Stream stream(std::span(const_cast<Frame &>(frame).data.data(), frame.length));
    return T::decode(stream);
}

/**
 * @brief Decodes a frame.
 *
 * @param node_id the expected node ID
 * @param frame the frame to decode
 * @return a T if successful; otherwise std::nullopt if the node or packet ID didn't match, or stream decoding failed
 */
template <Decodable T>
std::optional<T> decode(std::uint8_t node_id, const Frame &frame) {
    if (!frame.is_extended()) {
        return std::nullopt;
    }
    const auto ext_id = frame.extended_id();
    if ((ext_id & 0xff) != node_id) {
        return std::nullopt;
    }
    const auto packet_id = (ext_id >> 8) & 0x3ffff;
    if (packet_id != T::packet_id()) {
        return std::nullopt;
    }
    return decode_fast<T>(frame);
}

/**
 * @brief Builds a CAN frame from the given encodable data object and queues it for placement into a transmission
 * mailbox. The transmitted frame has a 29-bit extended identifier with the following format:
 *   PRIO[2:0] | PACKET_ID[17:0] | NODE_ID[7:0]
 *
 * Priority is ascending, unlike in the final CAN frame format, meaning 0 has the lowest arbitration priority, and
 * 7 has the highest priority.
 *
 * @param node_id 8-bit node ID
 * @param data the data payload to encode into the frame
 * @param priority 3-bit frame priority for arbitration
 */
template <Encodable T>
void transmit(std::uint8_t node_id, const T &data, std::uint8_t priority = T::default_priority()) {
    // Invert the priority direction since 0 is highest priority in CAN.
    priority = ~priority & 0b111u;

    // Build extended identifier with 3 bits of priority, 18 bits of packet ID, and 8 bits of source node ID.
    const auto ext_id =
        (static_cast<std::uint32_t>(priority) << 26) | (static_cast<std::uint32_t>(T::packet_id()) << 8) | node_id;
    Frame frame{
        .identifier{ExtendedIdentifier(ext_id)},
    };

    // Try to encode the data into the frame's data array.
    util::Stream stream(frame.data);
    if (data.encode(stream)) {
        frame.length = static_cast<std::uint8_t>(stream.head());
        queue_frame(frame);
    } else {
        assert(false);
    }
}

/**
 * @brief Sets up a reception filter to listen for the given packet in a frame with the given node ID.
 *
 * @param node_id the node ID to listen for
 * @param fifo the FIFO index to route to; must be 0 or 1
 * @param filter the filter index; must be in the range [0, 13]
 */
template <HasId T, void (*Callback)(const T &)>
void listen(std::uint8_t node_id, std::uint8_t fifo, std::uint8_t filter) {
    const auto expected =
        (static_cast<std::uint32_t>(T::packet_id()) << 11) | (static_cast<std::uint32_t>(node_id) << 3) | 0b100u;
    route_filter(fifo, filter, 0x1fffffff, expected);
    set_rx_callback(
        fifo, filter, +[](const Frame &frame) {
            if (const auto decoded = decode_fast<T>(frame)) {
                Callback(*decoded);
            }
        });
}

} // namespace can
