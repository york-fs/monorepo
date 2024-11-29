#include <config.hh>
#include <hal.hh>
#include <stm32f103xb.h>

int main() {
    hal::init_clocks();
    hal::init_can();

    // Configure PA1 LED pin.
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    hal::configure_gpio(GPIOA, 1, hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max10);

    while (true) {
        // Toggle LED.
        GPIOA->ODR ^= 0b10u;

        // Wait for TX ready on mailbox 0.
        hal::wait_equal(CAN1->TSR, CAN_TSR_TME0, CAN_TSR_TME0);

        const auto ext_id = (0x2aul << 8u) | config::k_dti_can_id;
        auto &mailbox = CAN1->sTxMailBox[0];
        mailbox.TIR = (ext_id << CAN_TI0R_EXID_Pos) | CAN_TI0R_IDE;
        mailbox.TDTR = 8;
        mailbox.TDLR = 0x5e240000;
        mailbox.TDHR = 0x86017100;

        // Request transmission.
        mailbox.TIR |= CAN_TI0R_TXRQ;

        hal::delay_ms(1000);
    }
}
