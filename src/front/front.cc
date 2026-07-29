#include <can.hh>
#include <config.hh>
#include <freertos.hh>
#include <front/apps.hh>
#include <front/can_messages.hh>
#include <hal.hh>
#include <rear/can_messages.hh>
#include <stm32f103xb.h>

#include <array>
#include <cstdint>

// TODO: Task notification wrapper which can set bits via FlagBitset.

using namespace front;

namespace {

/**
 * @brief Whether to enable the SWD debug logging task.
 */
constexpr bool k_enable_debug_logs = false;

/**
 * @brief Status sending period in milliseconds.
 */
constexpr std::uint32_t k_status_period = 1000;

/**
 * @brief Throttle sensing period in milliseconds.
 */
constexpr std::uint32_t k_throttle_period = 10;

enum class ThrottleState {
    AwaitingTractive,
    AwaitingActivation,
    Active,
};

std::array<std::uint16_t, 2> s_adc_buffer;

freertos::Task<128> s_main_task;
freertos::Task<2048> s_throttle_task;
freertos::Task<128> s_precharge_task;
freertos::Task<128> s_swd_task;

// Dashboard buttons with indicators.
hal::Gpio s_ts_button(hal::GpioPort::B, 1);
hal::Gpio s_ts_button_led(hal::GpioPort::B, 2);
hal::Gpio s_rtd_button(hal::GpioPort::C, 14);
hal::Gpio s_rtd_button_led(hal::GpioPort::C, 13);

hal::Gpio s_led(hal::GpioPort::B, 4);

void main_task(void *) {
    // Initialise CAN on port B.
    can::init(can::Port::B, config::k_can_speed, 1);

    // Enable CAN IRQs.
    hal::irq_enable(CAN1_RX0_IRQn, 7);
    hal::irq_enable(CAN1_TX_IRQn, 6);
    hal::irq_enable(CAN1_SCE_IRQn, 5);

    // Sequence the APPS sampling.
    hal::adc_init(ADC1, 2);
    hal::adc_init_dma(s_adc_buffer);
    hal::adc_sequence_channel(ADC1, 1, 7, 0b010u);
    hal::adc_sequence_channel(ADC1, 2, 8, 0b010u);

    // Enable automatic continuous ADC sampling.
    ADC1->CR2 |= ADC_CR2_CONT;
    hal::adc_start(ADC1);

    s_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

    freertos::PeriodScheduler scheduler;
    while (true) {
        s_led.write(!s_led.read());
        scheduler.delay_until_ms(k_status_period);
    }
}

void precharge_task(void *) {
    // Configure TS activate button input and associated indicator LED.
    s_ts_button.configure(hal::GpioInputMode::Floating);
    s_ts_button_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

    // Enable an external interrupt for the button.
    AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI1_PB;
    EXTI->IMR |= EXTI_IMR_MR1;
    EXTI->FTSR |= EXTI_FTSR_FT1;
    hal::irq_enable(EXTI1_IRQn, 8);

    // TODO: Toggle real precharge activation and check status over CAN.
    bool activation = false;
    while (true) {
        if (activation) {
            hal::gpio_set(s_ts_button_led);

            // Wait indefinitely for a disable request.
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            activation = false;

            // Notify throttle task.
            xTaskNotify(*s_throttle_task, 1u << 0, eSetBits);
        } else {
            s_ts_button_led.write(!s_ts_button_led.read());
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500)) != 0) {
                activation = true;
                xTaskNotify(*s_throttle_task, 1u << 1, eSetBits);
            }
        }
    }
}

void throttle_task(void *) {
    // Configure ready to drive button input and associated indicator LED.
    s_rtd_button.configure(hal::GpioInputMode::Floating);
    s_rtd_button_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

    // Enable an external interrupt for the button.
    AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI14_PC;
    EXTI->IMR |= EXTI_IMR_MR14;
    EXTI->FTSR |= EXTI_FTSR_FT14;
    hal::irq_enable(EXTI15_10_IRQn, 8);

    freertos::PeriodScheduler scheduler;
    std::array<Sensor, 2> sensors;

    // Calibrate based on the first sensor.
    // TODO: Save and load calibration data, it shouldn't be done on every start.
    Calibrator calibrator;
    while (!calibrator.update(s_adc_buffer[0])) {
        sensors[0].update_limits(s_adc_buffer[0]);
        sensors[1].update_limits(s_adc_buffer[1]);
        scheduler.delay_until_ms(k_throttle_period);
    }

    // Create a default throttle map.
    auto throttle_map = ThrottleMap::create_default();

    std::uint32_t led_counter = 0;
    auto state = ThrottleState::AwaitingTractive;
    while (true) {
        std::uint32_t notification = 0;
        if (xTaskNotifyWait(0, UINT32_MAX, &notification, 0) == pdTRUE) {
            // Received a task notification.
            if ((notification & (1u << 0)) != 0) {
                // TS disabled, go back to initial state.
                state = ThrottleState::AwaitingTractive;
            }
            if ((notification & (1u << 1)) != 0 && state == ThrottleState::AwaitingTractive) {
                // TS enabled.
                state = ThrottleState::AwaitingActivation;
            }
            if ((notification & (1u << 2)) != 0) {
                if (state == ThrottleState::AwaitingActivation) {
                    // TODO: Need to check that brake is pressed at the same time!
                    // RTD enabled.
                    state = ThrottleState::Active;
                } else {
                    // RTD disabled.
                    state = ThrottleState::AwaitingActivation;
                }
            }
        }

        // Set button LED.
        if (state == ThrottleState::AwaitingActivation) {
            led_counter = (led_counter + 1) % 50;
            if (led_counter == 0) {
                s_rtd_button_led.write(!s_rtd_button_led.read());
            }
        } else {
            s_rtd_button_led.write(state == ThrottleState::Active);
        }

        // Read sensors and calculate a desired current.
        std::uint16_t desired_current = 0;
        if (state == ThrottleState::Active) {
            // TODO: Do proper plausibility cross checking as well as taking the minimum.
            // TODO: Deadzone.
            // TODO: Current preload.
            const auto current_1 = throttle_map(sensors[0].normalise(s_adc_buffer[0]).value_or(0));
            const auto current_2 = throttle_map(sensors[1].normalise(s_adc_buffer[1]).value_or(0));
            desired_current = std::min(current_1, current_2);
        }

        ThrottleMessage throttle_message{
            .desired_current = desired_current,
            .raw_1 = s_adc_buffer[0],
            .raw_2 = s_adc_buffer[1],
        };
        can::transmit(config::k_front_can_id, throttle_message);
        scheduler.delay_until_ms(k_throttle_period);
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

extern "C" void EXTI1_IRQHandler() {
    // Clear pending bit.
    EXTI->PR = EXTI_PR_PR1;

    // Notify precharge task of button press.
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(*s_precharge_task, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

extern "C" void EXTI15_10_IRQHandler() {
    // Clear pending bit.
    EXTI->PR = EXTI_PR_PR14;

    // Notify throttle task of button press.
    BaseType_t higher_priority_task_woken = pdFALSE;
    xTaskNotifyFromISR(*s_throttle_task, 1u << 2, eSetBits, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

void vApplicationIdleHook() {
    hal::enter_sleep_mode(hal::WakeupSource::Interrupt);
}

void app_main() {
    s_main_task.init(&main_task, "main", 3);
    s_precharge_task.init(&precharge_task, "precharge", 2);
    s_throttle_task.init(&throttle_task, "throttle", 1);
    if constexpr (k_enable_debug_logs) {
        s_swd_task.init(&swd_task, "swd", 0);
    }
    vTaskStartScheduler();
}
