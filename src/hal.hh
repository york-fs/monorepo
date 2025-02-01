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

void swd_putc(char ch);
__attribute__((format(printf, 1, 2))) int swd_printf(const char *format, ...);

inline void wait_equal(const volatile std::uint32_t &reg, std::uint32_t mask, std::uint32_t desired) {
    while ((reg & mask) != desired) {
        asm volatile("" ::: "memory");
    }
}

} // namespace hal
