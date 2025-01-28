#include <can.hh>

#include <hal.hh>
#include <stm32f103xb.h>

#include <array>
#include <cstdint>

namespace can {
namespace {

std::array<fifo_callback_t, 2> s_fifo_callbacks{};
std::array<std::uint16_t, 2> s_fifo_overrun_counter{};

void fifo_interrupt(const std::uint8_t fifo_index) {
    volatile std::uint32_t &fifo_reg = fifo_index == 1 ? CAN1->RF1R : CAN1->RF0R;
    const auto &mailbox = CAN1->sFIFOMailBox[fifo_index];

    if ((fifo_reg & CAN_RF0R_FOVR0) != 0u) {
        // FIFO overrun.
        fifo_reg |= CAN_RF0R_FOVR0;
        s_fifo_overrun_counter[fifo_index]++;
    }

    const auto callback = s_fifo_callbacks[fifo_index];
    const auto pending_count = (fifo_reg & CAN_RF0R_FMP0_Msk) >> CAN_RF0R_FMP0_Pos;
    for (std::uint32_t i = 0; i < pending_count; i++) {
        // Read data from mailbox.
        Message message{
            .identifier = (mailbox.RIR & CAN_RI0R_EXID_Msk) >> CAN_RI0R_EXID_Pos,
            .data_low = mailbox.RDLR,
            .data_high = mailbox.RDHR,
            .length = static_cast<std::uint8_t>((mailbox.RDTR & CAN_RDT0R_DLC_Msk) >> CAN_RDT0R_DLC_Pos),
        };
        if (callback != nullptr) {
            callback(message);
        }

        // Release FIFO.
        fifo_reg |= CAN_RF0R_RFOM0;
    }
}

} // namespace

extern "C" void USB_LP_CAN1_RX0_IRQHandler() {
    fifo_interrupt(0);
}

extern "C" void CAN1_RX1_IRQHandler() {
    fifo_interrupt(1);
}

extern "C" void CAN1_SCE_IRQHandler() {
    if ((CAN1->MSR & CAN_MSR_ERRI) == 0u) {
        // Error interrupt bit not set, shouldn't really happen since we are not interrupting on wakeup or sleep
        // events, but ignore it anyway.
        return;
    }

    // Clear interrupt flag.
    CAN1->MSR |= CAN_MSR_ERRI;
}

void init() {
    // Ensure peripheral clocks are active.
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;

    // Remap CAN1 to PB8 and PB9.
    AFIO->MAPR |= AFIO_MAPR_CAN_REMAP_REMAP2;

    // Configure PB8 suitable to be CAN_RX and PB9 suitable to be CAN_TX.
    hal::configure_gpio(GPIOB, 8, hal::GpioInputMode::Floating);
    hal::configure_gpio(GPIOB, 9, hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max50);

    // Request CAN initialisation.
    CAN1->MCR |= CAN_MCR_INRQ;
    hal::wait_equal(CAN1->MSR, CAN_MSR_INAK, CAN_MSR_INAK);

    // Exit sleep mode.
    CAN1->MCR &= ~CAN_MCR_SLEEP;
    hal::wait_equal(CAN1->MSR, CAN_MSR_SLAK, 0u);

    // Set the bit timing register. Peripheral clocked at 16 MHz and 500 kbits/s.
    // The value below sets 13+2+1 (seg1+seg2+sync) time quanta per bit with a prescaler of 2.
    CAN1->BTR = 0x001c0001;

    // Set automatic bus-off management for now.
    // TODO: We should handle this manually eventually.
    CAN1->MCR |= CAN_MCR_ABOM;

    // Leave initialisation mode.
    CAN1->MCR &= ~CAN_MCR_INRQ;
    hal::wait_equal(CAN1->MSR, CAN_MSR_INAK, 0u);

    // Enable setting of CAN_MSR_ERRI on bus-off event.
    CAN1->IER |= CAN_IER_BOFIE;

    // Enable setting of CAN_MSR_ERRI on last error code change event.
    CAN1->IER |= CAN_IER_LECIE;

    // Enable message pending and overrun interrupt generation for both FIFOs.
    CAN1->IER |= CAN_IER_FOVIE0 | CAN_IER_FMPIE0;
    CAN1->IER |= CAN_IER_FOVIE1 | CAN_IER_FMPIE1;

    // Enable master error interrupt generation for any bit set in CAN_ESR.
    CAN1->IER |= CAN_IER_ERRIE;
}

void route_filter(const std::uint8_t filter, const std::uint8_t fifo, const std::uint32_t mask,
                  const std::uint32_t value) {
    const std::uint32_t filter_bit = 1u << filter;

    // Ensure filter is disabled.
    CAN1->FA1R &= ~filter_bit;

    // Enable filter init mode.
    CAN1->FMR |= CAN_FMR_FINIT;

    // Set 32-bit scale mask mode.
    CAN1->FM1R &= ~filter_bit;
    CAN1->FS1R |= filter_bit;

    // Set desired FIFO.
    if (fifo != 0u) {
        CAN1->FFA1R |= filter_bit;
    } else {
        CAN1->FFA1R &= ~filter_bit;
    }

    // Set mask and desired value.
    CAN1->sFilterRegister[filter].FR1 = value;
    CAN1->sFilterRegister[filter].FR2 = mask;

    // Enable the filter and leave init mode.
    CAN1->FA1R |= filter_bit;
    CAN1->FMR &= ~CAN_FMR_FINIT;
}

void set_fifo_callback(std::uint8_t index, fifo_callback_t callback) {
    s_fifo_callbacks[index] = callback;
}

bool transmit(const Message &message) {
    if ((CAN1->TSR & CAN_TSR_TME) == 0u) {
        // All mailboxes full.
        return false;
    }

    // Fill mailbox data.
    const auto mailbox_index = (CAN1->TSR & CAN_TSR_CODE_Msk) >> CAN_TSR_CODE_Pos;
    auto &mailbox = CAN1->sTxMailBox[mailbox_index];
    mailbox.TIR = (message.identifier << CAN_TI0R_EXID_Pos) | CAN_TI0R_IDE;
    mailbox.TDTR = message.length & 0xfu;
    mailbox.TDLR = message.data_low;
    mailbox.TDHR = message.data_high;

    // Request transmission.
    mailbox.TIR |= CAN_TI0R_TXRQ;
    return true;
}

} // namespace can
