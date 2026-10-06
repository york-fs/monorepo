#include <hal/gpio.hh>

#include <stm32f103xb.h>
#include <util/type_traits.hh>

#include <array>
#include <cstdint>

namespace hal::gpio {
namespace {

GPIO_TypeDef *gpio_for(Port port) {
    return std::array{
        GPIOA, GPIOB, GPIOC, GPIOD, GPIOE,
    }[util::to_underlying(port)];
}

void set_config(Descriptor descriptor, std::uint32_t bits) {
    auto *gpio = gpio_for(descriptor.port);
    const auto shift = (descriptor.pin % 8) * 4;
    auto &reg = descriptor.pin > 7 ? gpio->CRH : gpio->CRL;
    reg = (reg & ~(0xf << shift)) | (bits << shift);
}

} // namespace

void configure(Descriptor descriptor, InputMode input_mode) {
    std::uint32_t bits = 0;
    switch (input_mode) {
    case InputMode::Floating:
        bits |= 0b0100u;
        break;
    case InputMode::PullDown:
    case InputMode::PullUp:
        bits |= 0b1000u;
        break;
    }
    set_config(descriptor, bits);
    if (input_mode == InputMode::PullUp) {
        set(descriptor);
    } else if (input_mode == InputMode::PullDown) {
        reset(descriptor);
    }
}

void configure(Descriptor descriptor, OutputMode output_mode, SlewRate slew_rate) {
    std::uint32_t bits = 0;
    switch (output_mode) {
    case OutputMode::OpenDrain:
        bits |= 0b0100u;
        break;
    case OutputMode::AlternatePushPull:
        bits |= 0b1000u;
        break;
    case OutputMode::AlternateOpenDrain:
        bits |= 0b1100u;
        break;
    }
    switch (slew_rate) {
    case SlewRate::_10M:
        bits |= 0b01u;
        break;
    case SlewRate::_2M:
        bits |= 0b10u;
        break;
    case SlewRate::_50M:
        bits |= 0b11u;
        break;
    }
    set_config(descriptor, bits);
    reset(descriptor);
}

[[nodiscard]] bool read(Descriptor descriptor) {
    return (read_port(descriptor.port) & (1u << descriptor.pin)) != 0u;
}

void write(Descriptor descriptor, bool value) {
    value ? set(descriptor) : reset(descriptor);
}

[[nodiscard]] std::uint32_t read_port(Port port) {
    return gpio_for(port)->IDR;
}

void set_port(Port port, std::uint16_t mask) {
    gpio_for(port)->BSRR = static_cast<std::uint32_t>(mask);
}

void reset_port(Port port, std::uint16_t mask) {
    gpio_for(port)->BRR = static_cast<std::uint32_t>(mask);
}

void toggle_port(Port port, std::uint16_t mask) {
    auto *gpio = gpio_for(port);
    const auto odr = gpio->ODR;
    gpio->BSRR = ((odr & mask) << 16) | (~odr & mask);
}

void lock_port(Port port, std::uint16_t mask) {
    auto *gpio = gpio_for(port);
    gpio->LCKR = GPIO_LCKR_LCKK | mask;
    gpio->LCKR = static_cast<std::uint32_t>(mask);
    gpio->LCKR = GPIO_LCKR_LCKK | mask;
    gpio->LCKR;
}

} // namespace hal::gpio
