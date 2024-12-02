#include <can.hh>
#include <config.hh>
#include <hal.hh>
#include <stm32f103xb.h>

int main() {
    // Initialise system clocks and SysTick so we can use delay_ms.
    hal::init_clocks();
    hal::init_sys_tick();

    // Initialise and start CAN peripheral.
    can::init();

    // Configure PA1 LED pin.
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    hal::configure_gpio(GPIOA, 1, hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max10);

    while (true) {
        // Toggle LED.
        GPIOA->ODR ^= 0b10u;

        can::Message message{
            .identifier = (0x2aul << 8u) | config::k_dti_can_id,
            .data_low = 0x5e240000,
            .data_high = 0x86017100,
            .length = 8,
        };
        can::transmit(message);

        hal::delay_ms(1000);
    }
}
