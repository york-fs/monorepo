#pragma once

#include <cstdint>
#include <optional>
#include <utility>

namespace hal::gpio {

/**
 * @brief Enumeration of available GPIO ports.
 */
enum class Port {
    A,
    B,
    C,
    D,
    E,
};

/**
 * @brief Enumeration of available GPIO input modes.
 */
enum class InputMode {
    /**
     * @brief Similar to [`Floating`](\ref Floating) but without any Schmitt trigger filtering.
     */
    Analog,

    /**
     * @brief The default high-impedance pin mode with no pull-up or pull-down resistors enabled.
     */
    Floating,

    /**
     * @brief Enables a weak pull-down to VSS.
     */
    PullDown,

    /**
     * @brief Enables a weak pull-up to VDD.
     */
    PullUp,
};

/**
 * @brief Enumeration of available GPIO output modes.
 */
enum class OutputMode {
    /**
     * @brief Standard configuration with high-side and low-side switching enabled.
     */
    PushPull,

    /**
     * @brief Standard configuration with low-side switching enabled, but high-side switching disabled.
     *
     * Setting this pin high results in the pin becoming high-impedance. The high-side PMOS is effectively disconnected.
     */
    OpenDrain,

    /**
     * @brief Alternate configuration for peripheral use with high-side and low-side switching enabled.
     */
    AlternatePushPull,

    /**
     * @brief Alternate configuration for peripheral use with low-side switching enabled, but high-side switching
     * disabled.
     */
    AlternateOpenDrain,
};

/**
 * @brief Enumeration of available slew rates for output pins.
 */
enum class SlewRate {
    /**
     * @brief 2 MHz maximum slew rate.
     */
    _2M,

    /**
     * @brief 10 MHz maximum slew rate.
     */
    _10M,

    /**
     * @brief 50 MHz maximum slew rate.
     */
    _50M,
};

/**
 * @brief A GPIO port and pin pair.
 */
struct Descriptor {
    const Port port;
    const std::uint8_t pin;
};

/**
 * @brief Configures a GPIO pin as an input with the specified mode.
 *
 * @param descriptor the pin to configure
 * @param input_mode the input mode to use
 */
void configure(Descriptor descriptor, InputMode input_mode);

/**
 * @brief Configures a GPIO pin as an output with the specified mode and slew rate.
 *
 * @param descriptor the pin to configure
 * @param output_mode the output mode to use
 * @param slew_rate the slew rate to use
 */
void configure(Descriptor descriptor, OutputMode output_mode, SlewRate slew_rate);

/**
 * @brief Reads the state of a single GPIO pin.
 *
 * @param descriptor the pin to read
 * @return true if the pin is logic high; false if logic low
 */
[[nodiscard]] bool read(Descriptor descriptor);

/**
 * @brief Sets or resets the state of a single GPIO pin depending on the given value.
 *
 * @param descriptor the pin to write
 * @param value true for logic high; false for logic low
 */
void write(Descriptor descriptor, bool value);

/**
 * @brief Reads the input data register of a GPIO port.
 *
 * @param port the GPIO port to read
 */
[[nodiscard]] std::uint32_t read_port(Port port);

/**
 * @brief Sets the pins of a GPIO port according to the given mask.
 *
 * @param port the GPIO port to set bits in
 * @param mask the pin mask
 */
void set_port(Port port, std::uint16_t mask);

/**
 * @brief Resets the pins of a GPIO port according to the given mask.
 *
 * @param port the GPIO port to reset bits in
 * @param mask the pin mask
 */
void reset_port(Port port, std::uint16_t mask);

/**
 * @brief Toggles the pins of a GPIO port according to the given mask.
 *
 * @param port the GPIO port to toggle bits in
 * @param mask the pin mask
 */
void toggle_port(Port port, std::uint16_t mask);

/**
 * @brief Locks the pins of a GPIO port according to the given mask.
 *
 * @param port the GPIO port to lock bits in
 * @param mask the pin mask
 */
void lock_port(Port port, std::uint16_t mask);

template <typename F, typename... Ts>
void for_each_port(F &&callback, Ts... descriptors) {
    std::uint32_t bitset = 0;
    std::optional<Port> port;
    for (const auto descriptor : {descriptors...}) {
        if (port != descriptor.port) {
            if (port) {
                callback(*port, std::exchange(bitset, 0));
            }
            port = descriptor.port;
        }
        bitset |= 1u << descriptor.pin;
    }
    if (port) {
        callback(*port, bitset);
    }
}

/**
 * @brief Sets a list of GPIO pins to a logic high state. Pins passed in a contiguous order on the same port will be
 * atomically set at the same instant.
 *
 * @param list a list of pins to set
 */
template <typename... Ts>
void set(Ts... descriptors) {
    for_each_port(set_port, descriptors...);
}

/**
 * @brief Resets a list of GPIO pins to a logic low state. Pins passed in a contiguous order on the same port will be
 * atomically reset at the same instant.
 *
 * @param list a list of pins to reset
 */
template <typename... Ts>
void reset(Ts... descriptors) {
    for_each_port(reset_port, descriptors...);
}

/**
 * @brief Toggles a list of GPIO pins by inverting their current state. Pins passed in a contiguous order on the same
 * port will be atomically set or reset at the same instant.
 *
 * @param list a list of pins to toggle
 */
template <typename... Ts>
void toggle(Ts... descriptors) {
    for_each_port(toggle_port, descriptors...);
}

/**
 * @brief Locks a list of GPIO pins and prevents them from being reconfigured.
 *
 * @param list a list of pins to lock
 */
template <typename... Ts>
void lock(Ts... descriptors) {
    for_each_port(lock_port, descriptors...);
}

} // namespace hal::gpio
