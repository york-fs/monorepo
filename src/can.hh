#pragma once

#include <cstdint>

namespace can {

struct Message {
    std::uint32_t identifier;
    std::uint32_t data_low;
    std::uint32_t data_high;
    std::uint8_t length;
};

using fifo_callback_t = void (*)(const Message &);

/**
 * Initialises the CAN1 peripheral to 500 kbits/s, with CAN_RX mapped to PB8 and CAN_TX mapped to PB9. Assumes
 * a 28 MHz APB1 clock.
 */
void init();

/**
 * Configures and enables the specified CAN filter to route incoming messages to the specified FIFO index. A
 * message will be matched if, after applying the given bitmask, it equals the specified value.
 *
 * Incoming messages are structured as follows:
 *   EXID[28:0] | IDE | RTR | 0
 *
 * @param filter the filter index to configure; must be in the range [0, 13]
 * @param fifo   the FIFO index; must be 0 or 1
 * @param mask   the bitmask to be applied to the incoming message (bitwise AND)
 * @param value  the expected value to which the incoming message is compared to, after masking
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

} // namespace can
