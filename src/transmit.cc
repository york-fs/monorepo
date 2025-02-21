#include <can.hh>
#include <config.hh>
#include <hal.hh>
#include <stm32f103xb.h>

void app_main() {
    // Initialise SysTick so we can use delay_ms.
    hal::init_sys_tick();

    // Initialise and start CAN peripheral.
    static_cast<void>(can::init());

    // Configure PA1 LED pin.
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    hal::configure_gpio(GPIOA, 1, hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max10);

    while (true) {
        // Toggle LED.
        GPIOA->ODR ^= 0b10u;

        const auto message_bytes = std::to_array<std::uint8_t>({0x00, 0x00, 0x24, 0x5e, 0x00, 0x71, 0x01, 0x86});
        can::transmit(can::build_raw((0x2aul << 8u) | config::k_dti_can_id, message_bytes));

        hal::delay_ms(1000);
    }
}
