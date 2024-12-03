#include <can.hh>
#include <hal.hh>
#include <stm32f103xb.h>

static void message_callback(const can::Message &message) {
    const auto dl = message.data_low;
    const auto dh = message.data_high;
    hal::swd_printf("(id: %x, l: %u) - %02x %02x %02x %02x %02x %02x %02x %02x\n", message.identifier, message.length,
                    (dl >> 0u) & 0xffu, (dl >> 8u) & 0xffu, (dl >> 16u) & 0xffu, (dl >> 24u) & 0xffu,
                    (dh >> 0u) & 0xffu, (dh >> 8u) & 0xffu, (dh >> 16u) & 0xffu, (dh >> 24u) & 0xffu);
}

int main() {
    hal::init_clocks();
    can::init();

    // Route all messages to FIFO 0.
    can::route_filter(0, 0, 0, 0);
    can::set_fifo_callback(0, message_callback);
    hal::enable_irq(USB_LP_CAN1_RX0_IRQn, 2);

    SCB->SCR |= SCB_SCR_SLEEPONEXIT_Pos;
    __WFI();
}
