#include "hal.hh"

namespace hal {

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

} // namespace hal
