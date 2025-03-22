#pragma once

#include <stm32f103xb.h>

#include <cstdint>
#include <span>
#include <utility>

namespace hal {

enum class GpioInputMode : std::uint32_t {
    Analog = 0b00u,
    Floating = 0b01u,
    PullDown,
    PullUp,
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

enum class GpioPort {
    A,
    B,
    C,
    D,
    E,
};

class Gpio {
    GPIO_TypeDef *const m_port;
    const std::uint8_t m_pin;

public:
    Gpio(GpioPort port, std::uint8_t pin);

    void configure(GpioInputMode mode) const;
    void configure(GpioOutputMode mode, GpioOutputSpeed speed) const;

    GPIO_TypeDef *port() const { return m_port; }
    std::uint8_t pin() const { return m_pin; }
};

template <typename... Ts>
void gpio_set(Ts... list) {
    GPIO_TypeDef *port = nullptr;
    std::uint32_t bitset = 0;
    for (const auto gpio : {list...}) {
        if (port != gpio.port()) {
            if (port != nullptr) {
                port->BSRR = std::exchange(bitset, 0);
            }
            port = gpio.port();
        }
        bitset |= 1u << gpio.pin();
    }
    if (port != nullptr) {
        port->BSRR = bitset;
    }
}

template <typename... Ts>
void gpio_reset(Ts... list) {
    GPIO_TypeDef *port = nullptr;
    std::uint16_t bitset = 0;
    for (const auto gpio : {list...}) {
        if (port != gpio.port()) {
            if (port != nullptr) {
                port->BRR = std::exchange(bitset, 0);
            }
            port = gpio.port();
        }
        bitset |= 1u << gpio.pin();
    }
    if (port != nullptr) {
        port->BRR = bitset;
    }
}

void enable_irq(IRQn_Type irq, std::uint32_t priority);
void disable_irq(IRQn_Type irq);

/**
 * Enables the SysTick timer at a period of 1 ms.
 */
void init_sys_tick();

/**
 * Spins until the SysTick timer has ticked by the specified number of times. Requires init_sys_tick() to have
 * previously been called.
 */
void delay_ms(std::uint32_t ms);

/**
 * Enables and calibrates the given ADC.
 *
 * @param adc the target ADC peripheral
 * @param channel_count the sequence length of the regular group
 */
void adc_init(ADC_TypeDef *adc, std::uint32_t channel_count);

/**
 * Enables DMA in a circular, memory-increment mode for ADC1. Note that ADC2 doesn't support DMA.
 *
 * @param data the DMA destination buffer
 */
void adc_init_dma(std::span<std::uint16_t> data);

/**
 * Sets the channel to be sequenced at the given index. Refer to the datasheet for sample time meaning.
 *
 * @param adc the target ADC peripheral
 * @param index the 1-based sequence index
 * @param channel the 0-based ADC channel number
 * @param sample_time 3-bit sample time
 */
void adc_sequence_channel(ADC_TypeDef *adc, std::uint32_t index, std::uint32_t channel, std::uint32_t sample_time);

/**
 * Issues a software start to the given ADC.
 *
 * @param adc the target ADC peripheral
 */
void adc_start(ADC_TypeDef *adc);

void swd_putc(char ch);
__attribute__((format(printf, 1, 2))) int swd_printf(const char *format, ...);

/**
 * Waits until the register ANDed with the given mask is equal to the desired value, or the timeout expires.

 * @param reg the MMIO register
 * @param mask a mask to AND the register's value with
 * @param desired the desired value
 * @param timeout a timeout in milliseconds
 * @return true if the register now equals the desired value; false otherwise
 */
// TODO: Make this nodiscard.
bool wait_equal(const volatile std::uint32_t &reg, std::uint32_t mask, std::uint32_t desired,
                std::uint32_t timeout = UINT32_MAX);

} // namespace hal
