#include <can.hh>
#include <config.hh>
#include <freertos.hh>
#include <front/apps.hh>
#include <front/can_messages.hh>
#include <front/shutdown.hh>
#include <hal.hh>
#include <node_status.hh>
#include <precharge/can_messages.hh>
#include <precharge/state.hh>
#include <rear/can_messages.hh>
#include <rear/shutdown.hh>
#include <stm32f103xb.h>
#include <time_tracked.hh>

#include <array>
#include <bit>
#include <cstdint>
#include <optional>

// TODO: RTD buzzer.
// TODO: Task notification wrapper which can set bits via FlagBitset.

using namespace front;

namespace {

/**
 * @brief Status sending period in milliseconds.
 */
constexpr std::uint32_t k_status_period = 100;

/**
 * @brief Throttle sensing period in milliseconds.
 */
constexpr std::uint32_t k_throttle_period = 10;

/**
 * @brief Hard-coded value of the 3V3 rail powering the STM's ADC in 1 mV resolution.
 */
constexpr std::uint32_t k_mcu_vref = 3300;

TimeTracked<precharge::State> s_precharge_state(25);
TimeTracked<rear::StatusMessage> s_rear_status(25);
std::array<std::uint16_t, 9> s_adc_buffer;

freertos::Task<128> s_main_task;
freertos::Task<2048> s_throttle_task;
freertos::Task<128> s_debounce_task;
freertos::Task<128> s_led_task;
freertos::Task<128> s_swd_task;

// Shutdown inputs.
hal::Gpio s_sdn_estop(hal::GpioPort::B, 12);
hal::Gpio s_sdn_bots(hal::GpioPort::B, 13);
hal::Gpio s_sdn_inertia(hal::GpioPort::B, 14);
hal::Gpio s_sdn_aux(hal::GpioPort::B, 15);

// General outputs.
hal::Gpio s_rtd_buzzer(hal::GpioPort::A, 8);
hal::Gpio s_led(hal::GpioPort::B, 4);

// Dashboard buttons with indicator LEDs.
hal::Gpio s_ts_button(hal::GpioPort::B, 1);
hal::Gpio s_ts_button_led(hal::GpioPort::B, 2);
hal::Gpio s_rtd_button(hal::GpioPort::C, 14);
hal::Gpio s_rtd_button_led(hal::GpioPort::C, 13);

void main_task(void *) {
    // Initialise CAN on port B.
    can::init(can::Port::B, config::k_can_speed, 4);

    // Setup CAN listeners.
    can::listen<precharge::StatusMessage, [](const precharge::StatusMessage &precharge_status) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        const auto previous = s_precharge_state.receive(precharge_status.state);
        if (!previous || *previous != precharge_status.state) {
            vTaskNotifyGiveFromISR(*s_led_task, &higher_priority_task_woken);
        }
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }>(config::k_precharge_can_id, 0);
    can::listen<rear::StatusMessage, [](const rear::StatusMessage &rear_status) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        const auto previous = s_rear_status.receive(rear_status);
        if (!previous || previous->rtd_prevention_flags.value() != rear_status.rtd_prevention_flags.value()) {
            vTaskNotifyGiveFromISR(*s_led_task, &higher_priority_task_woken);
        }
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }>(config::k_rear_can_id, 1);

    // Initialise periodic node status transmission.
    node_status::init(config::k_front_can_id);

    // Enable CAN IRQs.
    hal::irq_enable(CAN1_RX0_IRQn, 7);
    hal::irq_enable(CAN1_TX_IRQn, 6);
    hal::irq_enable(CAN1_SCE_IRQn, 5);

    // Sequence the fuse, APPS, and temperature sensor sampling.
    hal::adc_init(ADC1, 9);
    hal::adc_init_dma(s_adc_buffer);
    for (std::uint32_t i = 0; i < 9; i++) {
        hal::adc_sequence_channel(ADC1, i + 1, i, 0b111u);
    }
    hal::adc_sequence_channel(ADC1, 10, 16, 0b111u);

    // Enable continuous ADC sampling.
    ADC1->CR2 |= ADC_CR2_CONT;
    hal::adc_start(ADC1);

    // Configure shutdown sampling inputs.
    s_sdn_estop.configure(hal::GpioInputMode::Floating);
    s_sdn_bots.configure(hal::GpioInputMode::Floating);
    s_sdn_inertia.configure(hal::GpioInputMode::Floating);
    s_sdn_aux.configure(hal::GpioInputMode::Floating);

    // Configure outputs.
    s_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_rtd_buzzer.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

    // Configure TS button.
    s_ts_button.configure(hal::GpioInputMode::Floating);
    AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI1_PB;
    EXTI->IMR |= EXTI_IMR_MR1;
    EXTI->FTSR |= EXTI_FTSR_FT1;
    hal::irq_enable(EXTI1_IRQn, 8);

    // Configure RTD button.
    s_rtd_button.configure(hal::GpioInputMode::Floating);
    AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI14_PC;
    EXTI->IMR |= EXTI_IMR_MR14;
    EXTI->FTSR |= EXTI_FTSR_FT14;
    hal::irq_enable(EXTI15_10_IRQn, 8);

    std::optional<TickType_t> ts_activation_desired;
    std::optional<TickType_t> rtd_activation_desired;
    freertos::PeriodScheduler scheduler;
    while (true) {
        // Handle dashboard button presses.
        std::uint32_t notification = 0;
        xTaskNotifyWait(0, UINT32_MAX, &notification, 0);
        if ((notification & (1u << 0)) != 0) {
            if (ts_activation_desired) {
                ts_activation_desired.reset();
            } else {
                ts_activation_desired.emplace(xTaskGetTickCount());
            }
        }
        if ((notification & (1u << 1)) != 0) {
            if (rtd_activation_desired) {
                rtd_activation_desired.reset();
            } else {
                rtd_activation_desired.emplace(xTaskGetTickCount());
            }
        }

        // Update data expiration timers.
        s_precharge_state.update();
        s_rear_status.update();
        if (!s_precharge_state || !s_rear_status) {
            // Update LED task since CAN messages are not being received.
            xTaskNotifyGive(*s_led_task);
        }

        // Desired state timeouts if the TS and RTD actual states don't activate in time.
        if (ts_activation_desired && xTaskGetTickCount() - *ts_activation_desired >= pdMS_TO_TICKS(100) &&
            (!s_rear_status || s_rear_status->ts_prevention_flags.any_set())) {
            ts_activation_desired.reset();
        }
        if (rtd_activation_desired && xTaskGetTickCount() - *rtd_activation_desired >= pdMS_TO_TICKS(100) &&
            (!s_rear_status || s_rear_status->rtd_prevention_flags.any_set())) {
            rtd_activation_desired.reset();
        }

        // Build bitset of raw shutdown samples.
        ShutdownSamples shutdown_samples;
        if (s_sdn_estop.read()) {
            shutdown_samples.set(ShutdownSample::EmergencyStop);
        }
        if (s_sdn_bots.read()) {
            shutdown_samples.set(ShutdownSample::BrakeOverTravel);
        }
        if (s_sdn_inertia.read()) {
            shutdown_samples.set(ShutdownSample::InertiaSwitch);
        }
        if (s_sdn_aux.read()) {
            shutdown_samples.set(ShutdownSample::Auxiliary);
        }

        StatusMessage status_message{
            .shutdown_samples = shutdown_samples,
            .ts_activation_desired = ts_activation_desired.has_value(),
            .rtd_activation_desired = rtd_activation_desired.has_value(),
        };
        can::transmit(config::k_front_can_id, status_message);

        // Calculate LVS voltages by reversing the 5.7x divider on each.
        std::array<std::uint16_t, 7> fuse_voltages{};
        std::transform(s_adc_buffer.begin(), s_adc_buffer.end(), fuse_voltages.begin(), [](std::uint16_t adc_value) {
            return (((k_mcu_vref * adc_value) >> 12) * 57) / 10;
        });

        LvsSampleMessage1 lvs_sample_message_1{
            .rtd_voltage = fuse_voltages[0],
            .apps_1_voltage = fuse_voltages[1],
            .apps_2_voltage = fuse_voltages[2],
            .front_voltage = fuse_voltages[3],
        };
        can::transmit(config::k_front_can_id, lvs_sample_message_1);

        LvsSampleMessage2 lvs_sample_message_2{
            .dwin_voltage = fuse_voltages[4],
            .aux_1_voltage = fuse_voltages[5],
            .aux_2_voltage = fuse_voltages[6],
        };
        can::transmit(config::k_front_can_id, lvs_sample_message_2);

        // Update node status temperature.
        node_status::update((k_mcu_vref * s_adc_buffer[9]) >> 12);

        scheduler.delay_until_ms(k_status_period);
    }
}

void throttle_task(void *) {
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

    while (true) {
        // Read sensors and calculate a desired current.
        // TODO: Do proper plausibility cross checking as well as taking the minimum.
        // TODO: Deadzone.
        // TODO: Current preload.
        const auto current_1 = throttle_map(sensors[0].normalise(s_adc_buffer[0]).value_or(0));
        const auto current_2 = throttle_map(sensors[1].normalise(s_adc_buffer[1]).value_or(0));
        const auto desired_current = std::min(current_1, current_2);

        ThrottleMessage throttle_message{
            .desired_current = desired_current,
            .raw_1 = s_adc_buffer[0],
            .raw_2 = s_adc_buffer[1],
        };
        can::transmit(config::k_front_can_id, throttle_message);

        scheduler.delay_until_ms(k_throttle_period);
    }
}

void debounce_task(void *) {
    TickType_t last_ts_button_time = 0;
    TickType_t last_rtd_button_time = 0;
    while (true) {
        // Wait for either button to be pressed. Clearing on both entry and exit is important here.
        std::uint32_t notification = 0;
        xTaskNotifyWait(UINT32_MAX, UINT32_MAX, &notification, portMAX_DELAY);

        // Only trigger if the button has been held for a minimum period.
        vTaskDelay(pdMS_TO_TICKS(100));

        // Button triggered if it's still pressed after the delay period and hasn't been pressed twice in the same
        // second.
        if ((notification & (1u << 0)) != 0 && !s_ts_button.read() &&
            xTaskGetTickCount() - last_ts_button_time >= pdMS_TO_TICKS(1000)) {
            last_ts_button_time = xTaskGetTickCount();
            xTaskNotify(*s_main_task, 1u << 0, eSetBits);
        }
        if ((notification & (1u << 1)) != 0 && !s_rtd_button.read() &&
            xTaskGetTickCount() - last_rtd_button_time >= pdMS_TO_TICKS(1000)) {
            last_rtd_button_time = xTaskGetTickCount();
            xTaskNotify(*s_main_task, 1u << 1, eSetBits);
        }
    }
}

void led_task(void *) {
    s_ts_button_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_rtd_button_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Setup DMA channel 6 (mapped to TIM3_CH1) to drive the TS button LED.
    std::array<std::uint32_t, 10> ts_buffer{};
    DMA1_Channel6->CPAR = std::bit_cast<std::uint32_t>(&GPIOB->BSRR);
    DMA1_Channel6->CMAR = std::bit_cast<std::uint32_t>(ts_buffer.data());
    DMA1_Channel6->CCR = DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_DIR;

    // Setup DMA channel 2 (mapped to TIM3_CH3) to drive the RTD button LED.
    std::array<std::uint32_t, 10> rtd_buffer{};
    DMA1_Channel2->CPAR = std::bit_cast<std::uint32_t>(&GPIOC->BSRR);
    DMA1_Channel2->CMAR = std::bit_cast<std::uint32_t>(rtd_buffer.data());
    DMA1_Channel2->CCR = DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_DIR;

    // Configure time-base to a 5 Hz period.
    TIM3->PSC = 1999;
    TIM3->ARR = 2799;

    // Enable DMA request generation on channel 1 and 3 output comparisons.
    TIM3->DIER = TIM_DIER_CC3DE | TIM_DIER_CC1DE;

    // Enable both channels.
    TIM3->CCER = TIM_CCER_CC3E | TIM_CCER_CC1E;

    // Enable counter.
    TIM3->CR1 = TIM_CR1_CEN;

    xTaskNotifyGive(xTaskGetCurrentTaskHandle());
    while (true) {
        // Wait for a state change.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Set TS button LED.
        DMA1_Channel6->CCR &= ~DMA_CCR_EN;
        if (!s_precharge_state) {
            // Off.
            ts_buffer[0] = 1u << (s_ts_button_led.pin() + 16);
            DMA1_Channel6->CNDTR = 1;
        } else if (s_precharge_state == precharge::State::Active) {
            // Solid.
            ts_buffer[0] = 1u << s_ts_button_led.pin();
            DMA1_Channel6->CNDTR = 1;
        } else {
            // Slow flash for standby and fast for everything else.
            const auto count = s_precharge_state == precharge::State::Standby ? 5 : 1;
            for (std::uint32_t i = 0; i < count; i++) {
                ts_buffer[i] = 1u << s_ts_button_led.pin();
                ts_buffer[count + i] = 1u << (s_ts_button_led.pin() + 16);
            }
            DMA1_Channel6->CNDTR = count * 2;
        }
        DMA1_Channel6->CCR |= DMA_CCR_EN;

        // Set RTD button LED.
        DMA1_Channel2->CCR &= ~DMA_CCR_EN;
        if (!s_precharge_state || *s_precharge_state != precharge::State::Active) {
            // Off.
            rtd_buffer[0] = 1u << (s_rtd_button_led.pin() + 16);
            DMA1_Channel2->CNDTR = 1;
        } else if (s_rear_status && s_rear_status->rtd_prevention_flags.none_set()) {
            // Solid.
            rtd_buffer[0] = 1u << s_rtd_button_led.pin();
            DMA1_Channel2->CNDTR = 1;
        } else {
            // Slow flash to indicate ready to activate, fast for any additional errors set.
            const auto count =
                (s_rear_status && s_rear_status->rtd_prevention_flags.only_set(rear::RtdPreventionFlag::NotRequested))
                    ? 5
                    : 1;
            for (std::uint32_t i = 0; i < count; i++) {
                rtd_buffer[i] = 1u << s_rtd_button_led.pin();
                rtd_buffer[count + i] = 1u << (s_rtd_button_led.pin() + 16);
            }
            DMA1_Channel2->CNDTR = count * 2;
        }
        DMA1_Channel2->CCR |= DMA_CCR_EN;
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

    // Notify debounce task of button press.
    BaseType_t higher_priority_task_woken = pdFALSE;
    xTaskNotifyFromISR(*s_debounce_task, 1u << 0, eSetBits, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

extern "C" void EXTI15_10_IRQHandler() {
    // Clear pending bit.
    EXTI->PR = EXTI_PR_PR14;

    // Notify debounce task of button press.
    BaseType_t higher_priority_task_woken = pdFALSE;
    xTaskNotifyFromISR(*s_debounce_task, 1u << 1, eSetBits, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

void vApplicationIdleHook() {
    hal::enter_sleep_mode(hal::WakeupSource::Interrupt);
}

void app_main() {
    s_main_task.init(&main_task, "main", 5);
    s_throttle_task.init(&throttle_task, "throttle", 3);
    s_debounce_task.init(&debounce_task, "debounce", 2);
    s_led_task.init(&led_task, "led", 1);
    if constexpr (config::enable_debug_logs()) {
        s_swd_task.init(&swd_task, "swd", 0);
    }
    vTaskStartScheduler();
}
