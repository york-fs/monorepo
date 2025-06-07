#include <can.hh>
#include <hal.hh>
#include <stm32f103xb.h>

static void message_callback(const can::Message &message) {
    hal::swd_printf("(id: %x, l: %u) - %02x %02x %02x %02x %02x %02x %02x %02x\n", message.identifier, message.length,
                    message.data[0], message.data[1], message.data[2], message.data[3], message.data[4],
                    message.data[5], message.data[6], message.data[7]);
}

void app_main() {
    static_cast<void>(can::init(can::Port::B));

    // Route all messages to FIFO 0.
    can::route_filter(0, 0, 0, 0);
    can::set_fifo_callback(0, message_callback);
    hal::enable_irq(USB_LP_CAN1_RX0_IRQn, 2);

    SCB->SCR |= SCB_SCR_SLEEPONEXIT_Pos;
    __WFI();
}
