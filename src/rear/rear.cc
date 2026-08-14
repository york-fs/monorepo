#include <can.hh>
#include <config.hh>
#include <freertos.hh>
#include <hal.hh>
#include <precharge/can_messages.hh>
#include <rear/can_messages.hh>
#include <stm32f103xb.h>
#include <time_tracked.hh>

#include <bit>
#include <cstdint>
#include <span>

using namespace rear;

namespace {

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

// Latest received statuses from other components.
TimeTracked<precharge::StatusMessage> s_precharge_status(25);

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
    can::listen<precharge::StatusMessage, [](const precharge::StatusMessage &precharge_status) {
        s_precharge_status.receive(precharge_status);
    }>(config::k_precharge_can_id, 0);

    // Enable CAN IRQs.
    hal::irq_enable(CAN1_RX0_IRQn, 7);
    hal::irq_enable(CAN1_TX_IRQn, 6);
    hal::irq_enable(CAN1_SCE_IRQn, 5);

    // Configure analog inputs.
    for (std::uint32_t i = 0; i < 8; i++) {
        hal::Gpio(hal::GpioPort::A, i).configure(hal::GpioInputMode::Analog);
    }
    for (std::uint32_t i = 0; i < 2; i++) {
        hal::Gpio(hal::GpioPort::B, i).configure(hal::GpioInputMode::Analog);
    }

    hal::adc_init(ADC1, 10);
    for (std::uint32_t i = 0; i < 10; i++) {
        hal::adc_sequence_channel(ADC1, i, i, 0b010u);
    }

    std::array<std::uint16_t, 10> adc_buffer{};
    hal::adc_init_dma(adc_buffer);

    while (true) {
        for (std::uint32_t i = 0; i < adc_buffer.size(); i++) {
            const auto voltage = (((k_mcu_vref * adc_buffer[i]) >> 12) * 57) / 10;
            hal::swd_printf("%u: %u\n", i, voltage);
        }

        rear::FlashMessage flash_message{};
        can::transmit(config::k_rear_can_id, flash_message);

        // Update all message expiry detections.
        s_precharge_status.update();

        hal::adc_start(ADC1);
        vTaskDelay(pdMS_TO_TICKS(1000));
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

        // Build a telemetry frame with the data offset by one byte to allow for the first COBS code. The size is also
        // limited to 3 bytes fewer to allow for the overall COBS overhead.
        util::Stream stream(std::span<std::uint8_t>(tx_bytes).subspan(1, 253));

        // Append general information.
        stream.write_be(freertos::uptime_ms());
        stream.write_byte(missed_tx_count);

        // Append online statuses for each component.
        stream.write_byte(s_precharge_status.has_value());

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

extern "C" void DMA1_Channel4_IRQHandler() {
    // Clear the pending flag and disable the channel.
    DMA1->IFCR |= DMA_IFCR_CTCIF4;
    DMA1_Channel4->CCR &= ~DMA_CCR_EN;
}

void vApplicationIdleHook() {
    hal::enter_sleep_mode(hal::WakeupSource::Interrupt);
}

void app_main() {
    s_main_task.init(&main_task, "main", 3);
    s_radio_task.init(&radio_task, "radio", 1);
    if constexpr (config::enable_debug_logs()) {
        s_swd_task.init(&swd_task, "swd", 0);
    }
    vTaskStartScheduler();
}
