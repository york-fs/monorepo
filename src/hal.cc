#include <hal.hh>

#include <miniprintf.h>
#include <stm32f103xb.h>

#include <cstdarg>
#include <cstdint>

namespace hal {

static volatile uint32_t s_ticks = 0;

extern "C" void SysTick_Handler() {
    s_ticks = s_ticks + 1;
}

static void set_gpio(GPIO_TypeDef *port, std::uint32_t pin, std::uint32_t cnf, std::uint32_t mode) {
    const auto shift = (pin % 8) * 4;
    auto &reg = pin > 7 ? port->CRH : port->CRL;
    reg &= ~(0xf << shift);
    reg |= cnf << (shift + 2);
    reg |= mode << shift;
}

void configure_gpio(GPIO_TypeDef *port, std::uint32_t pin, GpioInputMode mode) {
    set_gpio(port, pin, static_cast<std::uint32_t>(mode), 0b00u);
}

void configure_gpio(GPIO_TypeDef *port, std::uint32_t pin, GpioOutputMode mode, GpioOutputSpeed speed) {
    set_gpio(port, pin, static_cast<std::uint32_t>(mode), static_cast<std::uint32_t>(speed));
}

void enable_irq(IRQn_Type irq, std::uint32_t priority) {
    NVIC_SetPriority(irq, priority);
    NVIC_EnableIRQ(irq);
}

void disable_irq(IRQn_Type irq) {
    NVIC_DisableIRQ(irq);
}

void init_clocks() {
    // Enable HSE (8 MHz crystal) and wait for readiness.
    RCC->CR |= RCC_CR_HSEON;
    hal::wait_equal(RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY);

    // Configure PLL to HSE * 4 = 32 MHz.
    uint32_t rcc_cfgr = RCC->CFGR;
    rcc_cfgr &= ~RCC_CFGR_PLLMULL;
    rcc_cfgr |= RCC_CFGR_PLLMULL4;
    rcc_cfgr |= RCC_CFGR_PLLSRC;
    RCC->CFGR = rcc_cfgr;

    // Enable PLL and wait for readiness.
    RCC->CR |= RCC_CR_PLLON;
    hal::wait_equal(RCC->CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY);

    // Switch system clock to PLL. HSI is default, so no need to mask.
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    hal::wait_equal(RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL);

    // Done with the HSI, disable it.
    RCC->CR &= ~RCC_CR_HSION;
    hal::wait_equal(RCC->CR, RCC_CR_HSIRDY, 0u);

    // Set a 2x divider on APB1 clock (has CAN peripheral).
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
}

void init_sys_tick() {
    // Initialise SysTick at 1 ms.
    SysTick_Config(32000);
}

void delay_ms(std::uint32_t ms) {
    hal::wait_equal(s_ticks, 0xffffffffu, s_ticks + ms);
}

void swd_putc(char ch) {
    ITM_SendChar(ch);
}

int swd_printf(const char *format, ...) {
    va_list args;
    va_start(args, format);
    int rc = mini_vprintf_cooked(swd_putc, format, args);
    va_end(args);
    return rc;
}

} // namespace hal
