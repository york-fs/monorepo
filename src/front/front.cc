#include <can.hh>
#include <config.hh>
#include <freertos.hh>
#include <front/apps.hh>
#include <hal.hh>
#include <rear/can_messages.hh>

#include <atomic>

namespace {

/**
 * @brief Whether to enable the SWD debug logging task.
 */
constexpr bool k_enable_debug_logs = false;

/**
 * @brief Hard-coded value of the 3V3 rail powering the STM's ADC in 1 mV resolution.
 */
constexpr std::uint32_t k_mcu_vref = 3300;

std::atomic<bool> s_flash;

freertos::Task<256> s_main_task;
freertos::Task<128> s_throttle_task;
freertos::Task<128> s_button_task;
freertos::Task<128> s_swd_task;

hal::Gpio s_ts_button(hal::GpioPort::B, 1);
hal::Gpio s_ts_button_led(hal::GpioPort::B, 2);
hal::Gpio s_rtd_button(hal::GpioPort::B, 14);
hal::Gpio s_rtd_button_led(hal::GpioPort::B, 13);

hal::Gpio s_led(hal::GpioPort::B, 4);
hal::Gpio s_apps_1(hal::GpioPort::A, 7);
hal::Gpio s_apps_2(hal::GpioPort::B, 0);

void main_task(void *) {
    // Initialise CAN on port B.
    can::init(can::Port::B, config::k_can_speed, 1);
    can::listen<rear::FlashMessage, [](const rear::FlashMessage &) {
        s_flash.store(true);
    }>(config::k_rear_can_id, 0);

    // Enable CAN IRQs.
    hal::irq_enable(CAN1_RX0_IRQn, 6);
    hal::irq_enable(CAN1_SCE_IRQn, 5);

    s_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    while (true) {
        if (s_flash.exchange(false)) {
            s_led.write(!s_led.read());
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void throttle_task(void *) {
    s_apps_1.configure(hal::GpioInputMode::Analog);
    s_apps_2.configure(hal::GpioInputMode::Analog);

    hal::adc_init(ADC1, 2);
    hal::adc_sequence_channel(ADC1, 0, 7, 0b010u);
    hal::adc_sequence_channel(ADC1, 1, 8, 0b010u);

    std::array<std::uint16_t, 2> adc_buffer;
    hal::adc_init_dma(adc_buffer);

    while (true) {
        const auto left = (k_mcu_vref * adc_buffer[0]) >> 12;
        const auto right = (k_mcu_vref * adc_buffer[0]) >> 12;
        hal::swd_printf("l=%u, r=%u\n", left, right);

        hal::adc_start(ADC1);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void button_task(void *) {
    // Configure button inputs. These have external pull-ups.
    s_ts_button.configure(hal::GpioInputMode::Floating);
    s_rtd_button.configure(hal::GpioInputMode::Floating);

    // Configure LED outputs on the buttons.
    s_ts_button_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_rtd_button_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

    // TODO: Use interrupts.
    while (true) {
        s_ts_button_led.write(!s_ts_button.read());
        s_rtd_button_led.write(!s_rtd_button.read());

        vTaskDelay(pdMS_TO_TICKS(100));
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
    s_main_task.init(&main_task, "main", 3);
    s_throttle_task.init(&throttle_task, "throttle", 2);
    s_button_task.init(&button_task, "button", 1);
    if constexpr (k_enable_debug_logs) {
        s_swd_task.init(&swd_task, "swd", 0);
    }
    vTaskStartScheduler();
}
