#include <can.hh>
#include <config.hh>
#include <freertos.hh>
#include <hal.hh>
#include <rear/can_messages.hh>
#include <stm32f103xb.h>

#include <cstdint>
#include <string_view>

using namespace rear;

namespace {

/**
 * @brief Whether to enable the SWD debug logging task.
 */
constexpr bool k_enable_debug_logs = false;

/**
 * @brief Hard-coded value of the 3V3 rail powering the STM's ADC in 1 mV resolution.
 */
constexpr std::uint32_t k_mcu_vref = 3300;

hal::Gpio s_radio_tx(hal::GpioPort::A, 9);
hal::Gpio s_radio_rx(hal::GpioPort::A, 10);
hal::Gpio s_radio_cts(hal::GpioPort::A, 11);
hal::Gpio s_radio_rts(hal::GpioPort::A, 12);

freertos::Task<128> s_adc_task;
freertos::Task<128> s_radio_task;
freertos::Task<128> s_swd_task;

void adc_task(void *) {
    // Initialise CAN on port B.
    can::init(can::Port::B, config::k_can_speed, 1);

    // Enable CAN IRQs.
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

        hal::adc_start(ADC1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void radio_task(void *) {
    s_radio_cts.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    hal::gpio_set(s_radio_cts);

    // Configure GPIOs.
    s_radio_tx.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max2);
    s_radio_rx.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max2);

    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    USART1->CR1 |= USART_CR1_UE;
    USART1->BRR = 972;
    USART1->CR1 |= USART_CR1_RE | USART_CR1_TE;

    while (true) {
        std::string_view message("Hello world");
        for (char ch : message) {
            USART1->DR = ch;
            while (!(USART1->SR & USART_SR_TC)) {
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void swd_task(void *) {
    TickType_t last_schedule_time = xTaskGetTickCount();
    while (true) {
        xTaskDelayUntil(&last_schedule_time, pdMS_TO_TICKS(1000));

        hal::swd_printf("--------------------------------\n");
        hal::swd_printf("Uptime: %u\n", freertos::uptime_ms() / 1000);

        const auto can_stats = can::get_stats();
        hal::swd_printf("CAN status: %s %u/%u %u/%u\n", can::is_online() ? "online" : "offline", can_stats.rx_count,
                        can_stats.lost_rx_count, can_stats.tx_count, can_stats.lost_tx_count);
    }
}

} // namespace

void vApplicationIdleHook() {
    hal::enter_sleep_mode(hal::WakeupSource::Interrupt);
}

void app_main() {
    s_adc_task.init(&adc_task, "adc", 2);
    s_radio_task.init(&radio_task, "radio", 1);
    if constexpr (k_enable_debug_logs) {
        s_swd_task.init(&swd_task, "swd", 0);
    }
    vTaskStartScheduler();
}
