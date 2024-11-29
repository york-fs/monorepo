#pragma once

#include <stm32f103xb.h>

#include <cstdint>

namespace hal {

enum class GpioInputMode : std::uint32_t {
    Analog = 0b00u,
    Floating = 0b01u,
    PullUpPullDown = 0b10u,
};

enum class GpioOutputMode : std::uint32_t {
    PushPull = 0b00u,
    OpenDrain = 0b01u,
    AlternatePushPull = 0b10u,
    AlternateOpenDrain = 0b11u,
};

enum class GpioOutputSpeed : std::uint32_t {
    Max2 = 0b10u,
    Max10 = 0b01u,
    Max50 = 0b11u,
};

void configure_gpio(GPIO_TypeDef *port, std::uint32_t pin, GpioInputMode mode);
void configure_gpio(GPIO_TypeDef *port, std::uint32_t pin, GpioOutputMode mode, GpioOutputSpeed speed);
void delay_ms(std::uint32_t ms);

/**
 * Enables the 8 MHz external crystal oscillator and routes it to the PLL with a 4x multiplier, which then becomes the
 * main system and peripheral clocks. APB1 is configured to a 2x divider (16 MHz), and SysTick is configured to tick at
 * a 1 ms period.
 */
void init_clocks();

/**
 * Initialises the CAN1 peripheral to 500 kbits/s, with CAN_RX mapped to PB8 and CAN_TX mapped to PB9. Assumes
 * a 16 MHz APB1 clock.
 */
void init_can();

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
void route_can_filter(std::uint8_t filter, std::uint8_t fifo, std::uint32_t mask, std::uint32_t value);

void swd_putc(char ch);
__attribute__((format(printf, 1, 2))) int swd_printf(const char *format, ...);

inline void wait_equal(const volatile std::uint32_t &reg, std::uint32_t mask, std::uint32_t desired) {
    while ((reg & mask) != desired) {
        asm volatile("" ::: "memory");
    }
}

} // namespace hal
