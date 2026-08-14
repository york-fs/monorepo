#include <can.hh>
#include <config.hh>
#include <freertos.hh>
#include <front/can_messages.hh>
#include <hal.hh>
#include <precharge/can_messages.hh>
#include <rear/can_messages.hh>
#include <rear/fuse.hh>
#include <stm32f103xb.h>
#include <time_tracked.hh>

#include <bit>
#include <cstdint>
#include <span>

using namespace rear;

namespace {

/**
 * @brief The voltage threshold to consider a fuse working relative to the maximum measured LVS voltage in millivolts.
 */
constexpr std::uint16_t k_fuse_threshold = 1000;

/**
 * @brief The configured baud rate of the radio's UART.
 */
constexpr std::uint32_t k_radio_baud = 115200;

/**
 * @brief Telemetry radio send period in milliseconds.
 */
constexpr std::uint32_t k_radio_period = 100;

/**
 * @brief Hard-coded value of the 3V3 rail powering the STM's ADC in 1 mV resolution.
 */
constexpr std::uint32_t k_mcu_vref = 3300;

struct RadioData {
    FuseBitset fuse_bitset;
};

// Latest received statuses from other components.
TimeTracked<front::StatusMessage> s_front_status(250);
TimeTracked<precharge::StatusMessage> s_precharge_status(25);

// Front LVS voltages.
std::array<std::uint16_t, 7> s_front_lvs_voltages{};

// Queue for consistent radio data.
freertos::Queue<RadioData, 1> s_radio_data;

hal::Gpio s_radio_tx(hal::GpioPort::A, 9);
hal::Gpio s_radio_rx(hal::GpioPort::A, 10);
hal::Gpio s_radio_cts(hal::GpioPort::A, 11);
hal::Gpio s_radio_rts(hal::GpioPort::A, 12);

freertos::Task<128> s_main_task;
freertos::Task<256> s_radio_task;
freertos::Task<128> s_swd_task;

void main_task(void *) {
    // Initialise CAN on port B.
    can::init(can::Port::B, config::k_can_speed, 2);

    // Setup CAN listeners.
    can::listen<front::StatusMessage, [](const front::StatusMessage &front_status) {
        s_front_status.receive(front_status);
    }>(config::k_front_can_id, 0);
    can::listen<precharge::StatusMessage, [](const precharge::StatusMessage &precharge_status) {
        s_precharge_status.receive(precharge_status);
    }>(config::k_precharge_can_id, 1);
    can::listen<front::LvsSampleMessage1, [](const front::LvsSampleMessage1 &lvs_1) {
        s_front_lvs_voltages[0] = lvs_1.rtd_voltage;
        s_front_lvs_voltages[1] = lvs_1.apps_1_voltage;
        s_front_lvs_voltages[2] = lvs_1.apps_2_voltage;
        s_front_lvs_voltages[3] = lvs_1.front_voltage;
    }>(config::k_front_can_id, 2);
    can::listen<front::LvsSampleMessage2, [](const front::LvsSampleMessage2 &lvs_2) {
        s_front_lvs_voltages[4] = lvs_2.dwin_voltage;
        s_front_lvs_voltages[5] = lvs_2.aux_1_voltage;
        s_front_lvs_voltages[6] = lvs_2.aux_2_voltage;
    }>(config::k_front_can_id, 3);

    // Enable CAN IRQs.
    hal::irq_enable(CAN1_RX0_IRQn, 7);
    hal::irq_enable(CAN1_TX_IRQn, 6);
    hal::irq_enable(CAN1_SCE_IRQn, 5);

    hal::adc_init(ADC1, 10);
    for (std::uint32_t i = 0; i < 10; i++) {
        hal::adc_sequence_channel(ADC1, i + 1, i, 0b010u);
    }

    std::array<std::uint16_t, 10> adc_buffer{};
    hal::adc_init_dma(adc_buffer);

    DMA1_Channel1->CCR |= DMA_CCR_TCIE;
    hal::irq_enable(DMA1_Channel1_IRQn, 8);

    // Main loop which handles fuse and shutdown sampling, precharge heartbeat, and inverter control.
    freertos::PeriodScheduler scheduler;
    while (true) {
        scheduler.delay_until_ms(10);

        // Update all message expiry detections.
        s_front_status.update();
        s_precharge_status.update();

        // Sample all ADC channels.
        hal::adc_start(ADC1);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Create an array of rear and front measured fuse voltages.
        std::array<std::uint16_t, 17> fuse_voltages{};
        if (s_front_status) {
            freertos::in_critical_section([&] {
                auto out_it = std::next(fuse_voltages.begin(), 10);
                std::copy(s_front_lvs_voltages.begin(), s_front_lvs_voltages.end(), out_it);
            });
        }
        std::transform(adc_buffer.begin(), adc_buffer.end(), fuse_voltages.begin(), [](std::uint16_t adc_value) {
            return (k_mcu_vref * adc_value) >> 12;
        });

        // Reverse 5.7x divider on each measured fuse voltage.
        std::transform(fuse_voltages.begin(), fuse_voltages.end(), fuse_voltages.begin(), [](std::uint16_t voltage) {
            return (voltage * 57) / 10;
        });

        // Rate fuse soundness based on the highest voltage we measure.
        const auto max_voltage = *std::max_element(fuse_voltages.begin(), fuse_voltages.end());
        FuseBitset fuse_bitset;
        for (std::uint16_t fuse = 0; fuse < fuse_voltages.size(); fuse++) {
            if (max_voltage - fuse_voltages[fuse] <= k_fuse_threshold) {
                // The fuse is deemed working.
                fuse_bitset.set(Fuse(fuse));
            }
        }

        // Update radio data.
        s_radio_data.overwrite(RadioData{
            .fuse_bitset = fuse_bitset,
        });

        // TODO: Enum with TS and RTD off reason.

        if (!s_front_status || !s_precharge_status) {
            continue;
        }
        if (!s_front_status->ts_activation_desired) {
            continue;
        }
        can::transmit(config::k_precharge_can_id, precharge::HeartbeatMessage{});
    }
}

void radio_task(void *) {
    // Delay to allow the radio to leave its bootloader.
    vTaskDelay(pdMS_TO_TICKS(50));

    // Configure GPIOs.
    s_radio_tx.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max10);
    s_radio_rx.configure(hal::GpioInputMode::Floating);
    s_radio_cts.configure(hal::GpioInputMode::Floating);
    s_radio_rts.configure(hal::GpioInputMode::PullDown);

    // Enable peripheral clocks.
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // Configure DMA with the transfer complete interrupt enabled.
    std::array<std::uint8_t, 256> tx_bytes;
    DMA1_Channel4->CPAR = std::bit_cast<std::uint32_t>(&USART1->DR);
    DMA1_Channel4->CMAR = std::bit_cast<std::uint32_t>(tx_bytes.data());
    DMA1_Channel4->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE;

    // Program the configured baud rate.
    USART1->BRR = 56000000 / k_radio_baud;

    // Enable UART with DMA and transmission enabled.
    USART1->CR3 = USART_CR3_DMAT;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE;

    // Enable the interrupt.
    hal::irq_enable(DMA1_Channel4_IRQn, 7);

    // Keep track of the number of transmission windows we've missed.
    std::uint8_t missed_tx_count = 0;

    freertos::PeriodScheduler scheduler;
    while (true) {
        scheduler.delay_until_ms(k_radio_period);

        // Don't transmit if the radio's UART buffer is near full.
        if (s_radio_cts.read()) {
            missed_tx_count++;
            continue;
        }

        const auto data = *s_radio_data.receive(portMAX_DELAY);

        // Build a telemetry frame with the data offset by one byte to allow for the first COBS code. The size is also
        // limited to 3 bytes fewer to allow for the overall COBS overhead.
        util::Stream stream(std::span<std::uint8_t>(tx_bytes).subspan(1, 253));

        // Append general information.
        stream.write_be(freertos::uptime_ms());
        stream.write_byte(missed_tx_count);

        // Append online status bitset for each component.
        std::uint8_t online_bitset = 0;
        if (s_front_status) {
            online_bitset |= 1u << 0;
        }
        if (s_precharge_status) {
            online_bitset |= 1u << 2;
        }
        stream.write_byte(online_bitset);

        // Append distribution information.
        stream.write_be(data.fuse_bitset.value());

        // Append precharge information.
        stream.write_be(util::to_underlying(s_precharge_status->state));
        stream.write_be(s_precharge_status->error_flags.value());
        stream.write_be(s_precharge_status->precharge_voltage);
        stream.write_be(s_precharge_status->tractive_voltage);
        stream.write_be(s_precharge_status->relay_states.value());

        // Append a checksum.
        stream.write_be(freertos::in_critical_section([&] {
            return hal::crc_compute(stream.bytes());
        }));

        // COBS encode the data array in place.
        std::uint32_t write_index = 1;
        std::uint32_t code_index = 0;
        tx_bytes[code_index] = 1;
        for (std::uint32_t i = 1; i <= stream.head(); i++) {
            const std::uint8_t byte = tx_bytes[i];
            if (byte != 0) {
                tx_bytes[write_index++] = byte;
                tx_bytes[code_index]++;
            } else {
                code_index = write_index++;
                tx_bytes[code_index] = 1;
            }
        }

        // Append the frame delimiter.
        tx_bytes[write_index++] = 0;

        // Start the DMA transfer.
        USART1->SR = 0;
        DMA1_Channel4->CNDTR = write_index;
        DMA1_Channel4->CCR |= DMA_CCR_EN;
    }
}

void swd_task(void *) {
    freertos::PeriodScheduler scheduler;
    while (true) {
        scheduler.delay_until_ms(1000);
        hal::swd_printf("--------------------------------\n");
        hal::swd_printf("Uptime: %u\n", freertos::uptime_ms() / 1000);

        const auto can_stats = can::get_stats();
        hal::swd_printf("CAN status: %s %u/%u %u/%u\n", can::is_online() ? "online" : "offline", can_stats.rx_count,
                        can_stats.lost_rx_count, can_stats.tx_count, can_stats.lost_tx_count);
    }
}

} // namespace

extern "C" void DMA1_Channel1_IRQHandler() {
    BaseType_t higher_priority_task_woken = pdFALSE;
    DMA1->IFCR |= DMA_IFCR_CTCIF1;
    vTaskNotifyGiveFromISR(*s_main_task, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

extern "C" void DMA1_Channel4_IRQHandler() {
    // Clear the pending flag and disable the channel.
    DMA1->IFCR |= DMA_IFCR_CTCIF4;
    DMA1_Channel4->CCR &= ~DMA_CCR_EN;
}

void vApplicationIdleHook() {
    hal::enter_sleep_mode(hal::WakeupSource::Interrupt);
}

void app_main() {
    s_radio_data.init();
    s_main_task.init(&main_task, "main", 3);
    s_radio_task.init(&radio_task, "radio", 1);
    if constexpr (config::enable_debug_logs()) {
        s_swd_task.init(&swd_task, "swd", 0);
    }
    vTaskStartScheduler();
}
