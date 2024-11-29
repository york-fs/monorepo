#include <hal.hh>
#include <stm32f103xb.h>

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

void delay_ms(std::uint32_t ms) {
    hal::wait_equal(s_ticks, 0xffffffffu, s_ticks + ms);
}

} // namespace hal
