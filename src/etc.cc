#include <config.hh>
#include <dti.hh>
#include <hal.hh>
#include <stm32f103xb.h>

#include <cstdint>
#include <variant>

struct DtiHandler {
    void operator()(const dti::GeneralData1 &gd1) {
        hal::swd_printf("erpm=%d, duty=%d, dc_input=%d\n", gd1.erpm, gd1.duty_cycle, gd1.input_voltage);
    }

    void operator()(const dti::GeneralData2 &gd2) {
        hal::swd_printf("Iac=%d, Idc=%d\n", gd2.ac_current, gd2.dc_current);
    }

    void operator()(const dti::GeneralData3 &gd3) {
        hal::swd_printf("Tctl=%d, Tmot=%d, fault=%u", gd3.controller_temperature, gd3.motor_temperature,
                        static_cast<std::uint8_t>(gd3.fault_code));
    }

    void operator()(const dti::GeneralData5 &gd5) {
        hal::swd_printf("throttle=%d, brake=%d, pins=%x", gd5.throttle, gd5.brake, gd5.digital_pin_state);
    }

    void operator()(const dti::UnknownData &ud) {
        hal::swd_printf("unknown DTI data - %02x %02x %02x %02x %02x %02x %02x %02x\n", (ud.data_low >> 0u) & 0xffu,
                        (ud.data_low >> 8u) & 0xffu, (ud.data_low >> 16u) & 0xffu, (ud.data_low >> 24u) & 0xffu,
                        (ud.data_high >> 0u) & 0xffu, (ud.data_high >> 8u) & 0xffu, (ud.data_high >> 16u) & 0xffu,
                        (ud.data_high >> 24u) & 0xffu);
    }
};

int main() {
    hal::init_clocks();
    hal::init_can();

    // Route all incoming messages to FIFO 0.
    hal::route_can_filter(0, 0, 0x7feu, (config::k_dti_can_id << 3u) | 0b100u);

    // TODO: Use interrupts.
    DtiHandler dti_handler;
    while (true) {
        if ((CAN1->RF0R & CAN_RF0R_FMP0) == 0u) {
            continue;
        }

        const auto &mailbox = CAN1->sFIFOMailBox[0];
        const auto ext_id = (mailbox.RIR & CAN_RI0R_EXID_Msk) >> CAN_RI0R_EXID_Pos;
        const auto dti_packet = dti::parse_packet(ext_id, mailbox.RDLR, mailbox.RDHR);
        std::visit(dti_handler, dti_packet);

        // Release FIFO.
        CAN1->RF0R |= CAN_RF0R_RFOM0;
    }
}
