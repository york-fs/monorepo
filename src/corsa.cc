#include <can.hh>
#include <hal.hh>

void app_main() {
    hal::swd_printf("Hello world!\n");
    if (can::init(can::Port::B, can::Speed::_33_3)) {
        hal::swd_printf("CAN init successful\n");
    } else {
        hal::swd_printf("Failed to initialise CAN\n");
    }

    can::route_filter(0, 0, (0x7ffu << 21u) | 0b11u, 0x175u << 21u);
    can::set_fifo_callback(0, [](const can::Message &message) {
        hal::swd_printf("(id: %x, l: %u) - %02x %02x %02x %02x %02x %02x %02x %02x\n", message.standard_id().value,
                        message.length, message.data[0], message.data[1], message.data[2], message.data[3],
                        message.data[4], message.data[5], message.data[6], message.data[7]);
    });
    hal::enable_irq(USB_LP_CAN1_RX0_IRQn, 2);
}
