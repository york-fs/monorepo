#include <can.hh>

#include <hal.hh>
#include <stm32f103xb.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include <array>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <utility>

namespace can {
namespace {

// Allow 10 milliseconds for synchronising with the bus.
constexpr std::uint32_t k_init_timeout = 10;

QueueHandle_t s_tx_queue = nullptr;
TaskHandle_t s_task_handle = nullptr;
Speed s_speed;
std::array<rx_callback_t, 28> s_rx_callbacks{};
std::atomic<std::uint32_t> s_rx_count = 0;
std::atomic<std::uint32_t> s_tx_count = 0;
std::atomic<std::uint32_t> s_lost_rx_count = 0;
std::atomic<std::uint32_t> s_lost_tx_count = 0;
std::atomic<bool> s_online = false;

Identifier decode_identifier(std::uint32_t rir) {
    if ((rir & CAN_RI0R_IDE_Msk) != 0u) {
        return ExtendedIdentifier((rir & CAN_RI0R_EXID_Msk) >> CAN_RI0R_EXID_Pos);
    }
    return StandardIdentifier((rir & CAN_RI0R_STID_Msk) >> CAN_RI0R_STID_Pos);
}

void fifo_interrupt(const std::uint8_t fifo_index) {
    volatile std::uint32_t &fifo_reg = fifo_index == 1 ? CAN1->RF1R : CAN1->RF0R;
    if ((fifo_reg & CAN_RF0R_FOVR0) != 0u) {
        // FIFO overrun.
        fifo_reg |= CAN_RF0R_FOVR0;
        ++s_lost_rx_count;
    }

    const auto &mailbox = CAN1->sFIFOMailBox[fifo_index];
    while ((fifo_reg & CAN_RF0R_FMP0) != 0) {
        // Read data from mailbox.
        Frame frame{
            .identifier = decode_identifier(mailbox.RIR),
            .data{
                static_cast<std::uint8_t>(mailbox.RDLR & 0xffu),
                static_cast<std::uint8_t>((mailbox.RDLR >> 8u) & 0xffu),
                static_cast<std::uint8_t>((mailbox.RDLR >> 16u) & 0xffu),
                static_cast<std::uint8_t>((mailbox.RDLR >> 24u) & 0xffu),
                static_cast<std::uint8_t>(mailbox.RDHR & 0xffu),
                static_cast<std::uint8_t>((mailbox.RDHR >> 8u) & 0xffu),
                static_cast<std::uint8_t>((mailbox.RDHR >> 16u) & 0xffu),
                static_cast<std::uint8_t>((mailbox.RDHR >> 24u) & 0xffu),
            },
            .length = static_cast<std::uint8_t>((mailbox.RDTR & CAN_RDT0R_DLC_Msk) >> CAN_RDT0R_DLC_Pos),
        };
        const auto filter_index = (mailbox.RDTR & CAN_RDT0R_FMI_Msk) >> CAN_RDT0R_FMI_Pos;

        // Release the FIFO to allow more inbound frames.
        fifo_reg |= CAN_RF0R_RFOM0;

        const auto callback = s_rx_callbacks[(fifo_index * 14) + filter_index];
        if (callback != nullptr) {
            callback(frame);
        }
        ++s_rx_count;
    }
}

bool reinit(Speed speed) {
    // Request CAN initialisation.
    // TODO: Use vTaskDelayUntil for waiting.
    CAN1->MCR |= CAN_MCR_INRQ;
    if (!hal::wait_equal(CAN1->MSR, CAN_MSR_INAK, CAN_MSR_INAK, k_init_timeout)) {
        return false;
    }

    // Exit sleep mode.
    CAN1->MCR &= ~CAN_MCR_SLEEP;
    if (!hal::wait_equal(CAN1->MSR, CAN_MSR_SLAK, 0u, k_init_timeout)) {
        return false;
    }

    // Configure the bit timing register. The peripheral is clocked at 28 MHz.
    switch (speed) {
    case Speed::_33_3:
        // 33.333 kbit/s sets 12+2+3 (seg1+seg2+sync) time quanta per bit with a prescaler of 56.
        CAN1->BTR = 0x001b0037;
        break;
    case Speed::_500:
        // 500 kbit/s sets 11+2+1 (seg1+seg2+sync) time quanta per bit with a prescaler of 4.
        CAN1->BTR = 0x001a0003;
        break;
    case Speed::_1000:
        // 1000 kbit/s sets 11+2+1 (seg1+seg2+sync) time quanta per bit with a prescaler of 2.
        CAN1->BTR = 0x001a0001;
        break;
    }

    // Disable time triggered communication mode, automatic bus-off recovery, and automatic wakeup.
    CAN1->MCR &= ~(CAN_MCR_TTCM | CAN_MCR_ABOM | CAN_MCR_AWUM);

    // Enable automatic retransmission.
    CAN1->MCR &= ~CAN_MCR_NART;

    // Disable receive FIFO locking (enable overrun possibility).
    CAN1->MCR &= ~CAN_MCR_RFLM;

    // Prioritise transmissions by identifier.
    CAN1->MCR &= ~CAN_MCR_TXFP;

    // Enable error interrupts.
    CAN1->IER |= CAN_IER_ERRIE | CAN_IER_LECIE | CAN_IER_BOFIE | CAN_IER_EPVIE | CAN_IER_EWGIE;

    // Enable message pending and overrun interrupts for both FIFOs.
    CAN1->IER |= CAN_IER_FOVIE0 | CAN_IER_FMPIE0;
    CAN1->IER |= CAN_IER_FOVIE1 | CAN_IER_FMPIE1;

    // Enable transmit mailbox empty interrupt.
    CAN1->IER |= CAN_IER_TMEIE;

    // Leave initialisation mode.
    CAN1->MCR &= ~CAN_MCR_INRQ;
    return hal::wait_equal(CAN1->MSR, CAN_MSR_INAK, 0u, k_init_timeout) && (CAN1->ESR & CAN_ESR_BOFF) == 0;
}

/**
 * @brief The task which is responsible for placing frames into the TX mailboxes and reinitialising the peripheral.
 */
void tx_task(void *) {
    // Start with all 3 mailboxes free.
    xTaskNotify(s_task_handle, 3, eSetValueWithOverwrite);
    while (true) {
        if (!s_online.load()) {
            // TODO: Backoff with jitter.
            vTaskDelay(pdMS_TO_TICKS(100));
            s_online.store(reinit(s_speed));
            xTaskNotify(s_task_handle, 3, eSetValueWithOverwrite);
            continue;
        }

        // Wait for a frame to be queued.
        Frame frame;
        if (xQueueReceive(s_tx_queue, &frame, portMAX_DELAY) != pdPASS) {
            continue;
        }

        // Wait for at least one available mailbox and decrement the count.
        [[maybe_unused]] const auto free_count = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

        // Skip if we were woken due to bus off.
        if (!s_online.load()) {
            // We lose the frame we dequeued.
            ++s_lost_tx_count;
            continue;
        }

        // Check that there are the correct amount of free mailboxes.
        // TODO: Figure out why this assertion isn't always true in the error case.
        // assert(((CAN1->TSR & CAN_TSR_TME_Msk) >> CAN_TSR_TME_Pos) >= free_count);

        // Fill in a free mailbox.
        auto &mailbox = CAN1->sTxMailBox[(CAN1->TSR & CAN_TSR_CODE_Msk) >> CAN_TSR_CODE_Pos];
        if (frame.is_extended()) {
            mailbox.TIR = (frame.extended_id() << CAN_TI0R_EXID_Pos) | CAN_TI0R_IDE;
        } else {
            mailbox.TIR = frame.standard_id() << CAN_TI0R_STID_Pos;
        }
        mailbox.TDTR = frame.length & 0xfu;
        mailbox.TDLR = (static_cast<std::uint32_t>(frame.data[3]) << 24) |
                       (static_cast<std::uint32_t>(frame.data[2]) << 16) |
                       (static_cast<std::uint32_t>(frame.data[1]) << 8) | frame.data[0];
        mailbox.TDHR = (static_cast<std::uint32_t>(frame.data[7]) << 24) |
                       (static_cast<std::uint32_t>(frame.data[6]) << 16) |
                       (static_cast<std::uint32_t>(frame.data[5]) << 8) | frame.data[4];

        // Request transmission.
        mailbox.TIR |= CAN_TI0R_TXRQ;
    }
}

std::pair<hal::Gpio, hal::Gpio> pin_pair(Port port) {
    switch (port) {
    case Port::B:
        return std::make_pair(hal::Gpio(hal::GpioPort::B, 8), hal::Gpio(hal::GpioPort::B, 9));
    case Port::D:
        return std::make_pair(hal::Gpio(hal::GpioPort::D, 0), hal::Gpio(hal::GpioPort::D, 1));
    default:
        return std::make_pair(hal::Gpio(hal::GpioPort::A, 11), hal::Gpio(hal::GpioPort::A, 12));
    }
}

} // namespace

extern "C" void USB_HP_CAN1_TX_IRQHandler() {
    const std::uint32_t tsr = CAN1->TSR;
    std::uint32_t free_mailboxes = 0;
    if ((tsr & CAN_TSR_RQCP0) != 0) {
        CAN1->TSR |= CAN_TSR_RQCP0;
        if ((tsr & CAN_TSR_TXOK0) != 0) {
            ++s_tx_count;
        } else {
            ++s_lost_tx_count;
        }
        free_mailboxes++;
    }
    if ((tsr & CAN_TSR_RQCP1) != 0) {
        CAN1->TSR |= CAN_TSR_RQCP1;
        if ((tsr & CAN_TSR_TXOK1) != 0) {
            ++s_tx_count;
        } else {
            ++s_lost_tx_count;
        }
        free_mailboxes++;
    }
    if ((tsr & CAN_TSR_RQCP2) != 0) {
        CAN1->TSR |= CAN_TSR_RQCP2;
        if ((tsr & CAN_TSR_TXOK2) != 0) {
            ++s_tx_count;
        } else {
            ++s_lost_tx_count;
        }
        free_mailboxes++;
    }
    if (free_mailboxes != 0) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        xTaskNotifyFromISR(s_task_handle, free_mailboxes, eIncrement, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

extern "C" void USB_LP_CAN1_RX0_IRQHandler() {
    fifo_interrupt(0);
}

extern "C" void CAN1_RX1_IRQHandler() {
    fifo_interrupt(1);
}

extern "C" void CAN1_SCE_IRQHandler() {
    // Clear error interrupt flag.
    CAN1->MSR |= CAN_MSR_ERRI;

    // TODO: Record counts for each error type.

    // Update online status.
    s_online.store((CAN1->ESR & CAN_ESR_BOFF) == 0);
    if (!s_online.load()) {
        // Kick task notification so that it attempt to resync with the bus.
        xTaskNotifyFromISR(s_task_handle, 1, eIncrement, nullptr);
    }
}

void init(Port port, Speed speed, std::uint32_t task_priority, std::uint32_t tx_queue_size) {
    // Enable CAN1's peripheral clock.
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

    // Perform a full reset.
    RCC->APB1RSTR |= RCC_APB1RSTR_CAN1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN1RST;

    // Configure pin functions.
    const auto [rx_pin, tx_pin] = pin_pair(port);
    rx_pin.configure(hal::GpioInputMode::Floating);
    tx_pin.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max50);

    // Configure alternate function if not the default pin pair (port A).
    AFIO->MAPR &= ~AFIO_MAPR_CAN_REMAP;
    if (port == Port::B) {
        // Remap to PB8 and PB9.
        AFIO->MAPR |= AFIO_MAPR_CAN_REMAP_REMAP2;
    } else if (port == Port::D) {
        // Remap to PD0 and PD1.
        AFIO->MAPR |= AFIO_MAPR_CAN_REMAP_REMAP3;
    }

    // Create the queue of outbound frames.
    s_tx_queue = xQueueCreate(tx_queue_size, sizeof(Frame));

    // Create the task with the given priority.
    // TODO: Tune stack size.
    s_speed = speed;
    xTaskCreate(&tx_task, "can", 128, nullptr, task_priority, &s_task_handle);

    s_online.store(reinit(speed));
}

void set_rx_callback(std::uint8_t fifo, std::uint8_t filter, rx_callback_t callback) {
    s_rx_callbacks[(fifo * 14) + filter] = callback;
}

void route_filter(std::uint8_t fifo, std::uint8_t filter, std::uint32_t mask, std::uint32_t value) {
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

void queue_frame(const Frame &frame) {
    assert(s_tx_queue != nullptr);
    if (xQueueSendToBack(s_tx_queue, &frame, 0) != pdPASS) {
        ++s_lost_tx_count;
    }
}

bool is_online() {
    return s_online.load();
}

Stats get_stats() {
    return {
        .rx_count = s_rx_count.load(),
        .tx_count = s_tx_count.load(),
        .lost_rx_count = s_lost_rx_count.load(),
        .lost_tx_count = s_lost_tx_count.load(),
    };
}

} // namespace can
