#include <can.hh>
#include <config.hh>
#include <freertos.hh>
#include <hal.hh>
#include <rear/can_messages.hh>

#include <atomic>

namespace {

/**
 * @brief Whether to enable the SWD debug logging task.
 */
constexpr bool k_enable_debug_logs = false;

std::atomic<bool> s_flash;

freertos::Task<256> s_main_task;
freertos::Task<128> s_swd_task;

hal::Gpio s_led(hal::GpioPort::B, 4);

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

void vApplicationIdleHook() {
    hal::enter_sleep_mode(hal::WakeupSource::Interrupt);
}

void app_main() {
    s_main_task.init(&main_task, "main", 1);
    if constexpr (k_enable_debug_logs) {
        s_swd_task.init(&swd_task, "swd", 0);
    }
    vTaskStartScheduler();
}
