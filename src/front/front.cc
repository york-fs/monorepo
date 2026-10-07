#include <config.hh>
#include <freertos.hh>
#include <front/apps.hh>
#include <front/can_messages.hh>
#include <front/shutdown.hh>
#include <hal.hh>
#include <hal/can.hh>
#include <hal/gpio.hh>
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

using namespace front;

namespace {

/**
 * @brief The timeout to use for the TS and RTD actual activation state latching in milliseconds. This should be kept
 * well below the precharge's rate limit time.
 */
constexpr std::uint32_t k_activation_desired_timeout = 500;

/**
 * @brief The minimum duration a dashboard button must be held to register a press in milliseconds.
 */
constexpr std::uint32_t k_button_hold_duration = 100;

/**
 * @brief The duration to ignore subsequent button presses after a successful button press in milliseconds. This should
 * be kept longer than the activation desired timeout and the hold duration.
 */
constexpr std::uint32_t k_button_lockout_duration = 1000;

/**
 * @brief The duration to drive the RTD horn after activation in milliseconds.
 */
constexpr std::uint32_t k_rtd_horn_duration = 1000;

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

/**
 * @brief Shutdown sample input pins.
 */
constexpr hal::gpio::Descriptor k_sdn_estop(hal::gpio::Port::B, 12);
constexpr hal::gpio::Descriptor k_sdn_bots(hal::gpio::Port::B, 13);
constexpr hal::gpio::Descriptor k_sdn_inertia(hal::gpio::Port::B, 14);
constexpr hal::gpio::Descriptor k_sdn_aux(hal::gpio::Port::B, 15);

/**
 * @brief Horn and on-board LED outputs.
 */
constexpr hal::gpio::Descriptor k_rtd_horn(hal::gpio::Port::A, 8);
constexpr hal::gpio::Descriptor k_led(hal::gpio::Port::B, 4);

/**
 * @brief Dashboard button inputs and associated indicator LED outputs.
 */
constexpr hal::gpio::Descriptor k_ts_button(hal::gpio::Port::B, 1);
constexpr hal::gpio::Descriptor k_ts_button_led(hal::gpio::Port::B, 2);
constexpr hal::gpio::Descriptor k_rtd_button(hal::gpio::Port::C, 14);
constexpr hal::gpio::Descriptor k_rtd_button_led(hal::gpio::Port::C, 13);

TimeTracked<precharge::State> s_precharge_state(25);
TimeTracked<rear::StatusMessage> s_rear_status(25);
std::array<volatile std::uint16_t, 9> s_adc_buffer;

freertos::Task<128> s_main_task;
freertos::Task<2048> s_throttle_task;
freertos::Task<128> s_debounce_task;
freertos::Task<128> s_led_task;
freertos::Task<128> s_swd_task;

void main_task(void *) {
    // Initialise CAN on port B.
    hal::can::init(hal::can::Port::B, config::k_can_speed, 4);

    // Setup CAN listeners.
    hal::can::listen<precharge::StatusMessage, [](const precharge::StatusMessage &precharge_status) {
        freertos::InterruptYielder interrupt_yielder;
        const auto previous = s_precharge_state.receive(precharge_status.state);
        if (!previous || *previous != precharge_status.state) {
            s_led_task.notify_give_isr(0, interrupt_yielder);
        }
    }>(config::k_precharge_can_id, 0);
    hal::can::listen<rear::StatusMessage, [](const rear::StatusMessage &rear_status) {
        freertos::InterruptYielder interrupt_yielder;
        const auto previous = s_rear_status.receive(rear_status);
        if (!previous || previous->rtd_prevention_flags.value() != rear_status.rtd_prevention_flags.value()) {
            s_led_task.notify_give_isr(0, interrupt_yielder);
        }
    }>(config::k_rear_can_id, 1);

    // Configure shutdown sampling inputs.
    hal::gpio::configure(k_sdn_estop, hal::gpio::InputMode::Floating);
    hal::gpio::configure(k_sdn_bots, hal::gpio::InputMode::Floating);
    hal::gpio::configure(k_sdn_inertia, hal::gpio::InputMode::Floating);
    hal::gpio::configure(k_sdn_aux, hal::gpio::InputMode::Floating);

    // Configure outputs.
    hal::gpio::configure(k_rtd_horn, hal::gpio::OutputMode::PushPull, hal::gpio::SlewRate::_2M);
    hal::gpio::configure(k_led, hal::gpio::OutputMode::PushPull, hal::gpio::SlewRate::_2M);

    // Configure TS button.
    hal::gpio::configure(k_ts_button, hal::gpio::InputMode::Floating);
    AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI1_PB;
    EXTI->IMR |= EXTI_IMR_MR1;
    EXTI->FTSR |= EXTI_FTSR_FT1;

    // Configure RTD button.
    hal::gpio::configure(k_rtd_button, hal::gpio::InputMode::Floating);
    AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI14_PC;
    EXTI->IMR |= EXTI_IMR_MR14;
    EXTI->FTSR |= EXTI_FTSR_FT14;

    // Initialise periodic node status transmission.
    node_status::init(config::k_front_can_id);

    // Enable CAN and EXTI IRQs.
    hal::irq_enable(EXTI1_IRQn, 8);
    hal::irq_enable(EXTI15_10_IRQn, 8);
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

    std::optional<TickType_t> ts_activation_desired;
    std::optional<TickType_t> rtd_activation_desired;
    std::optional<TickType_t> rtd_activation_time;
    bool apps_calibrated = false;
    freertos::PeriodScheduler scheduler;
    while (true) {
        // Handle dashboard button presses.
        const auto notification = freertos::notify_wait(0, 0, UINT32_MAX, 0);
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
        if ((notification & (1u << 2)) != 0) {
            apps_calibrated = true;
        }

        // Update data expiration timers.
        s_precharge_state.update();
        s_rear_status.update();
        if (!s_precharge_state || !s_rear_status) {
            // Update LED task since CAN messages are not being received.
            s_led_task.notify_give(0);
        }

        // Desired state timeouts if the TS and RTD actual states don't activate in time.
        if (ts_activation_desired &&
            xTaskGetTickCount() - *ts_activation_desired >= pdMS_TO_TICKS(k_activation_desired_timeout) &&
            (!s_rear_status || s_rear_status->ts_prevention_flags.any_set())) {
            ts_activation_desired.reset();
        }
        if (rtd_activation_desired &&
            xTaskGetTickCount() - *rtd_activation_desired >= pdMS_TO_TICKS(k_activation_desired_timeout) &&
            (!s_rear_status || s_rear_status->rtd_prevention_flags.any_set())) {
            rtd_activation_desired.reset();
        }

        // Keep track of RTD activation time.
        if (s_rear_status && s_rear_status->rtd_prevention_flags.none_set() && rtd_activation_desired) {
            if (!rtd_activation_time) {
                rtd_activation_time.emplace(xTaskGetTickCount());
            }
        } else {
            rtd_activation_time.reset();
        }

        // Drive RTD horn.
        if (rtd_activation_time && xTaskGetTickCount() - *rtd_activation_time <= pdMS_TO_TICKS(k_rtd_horn_duration)) {
            hal::gpio::set(k_rtd_horn);
        } else {
            hal::gpio::reset(k_rtd_horn);
        }

        // Build bitset of raw shutdown samples.
        ShutdownSamples shutdown_samples;
        if (hal::gpio::read(k_sdn_estop)) {
            shutdown_samples.set(ShutdownSample::EmergencyStop);
        }
        if (hal::gpio::read(k_sdn_bots)) {
            shutdown_samples.set(ShutdownSample::BrakeOverTravel);
        }
        if (hal::gpio::read(k_sdn_inertia)) {
            shutdown_samples.set(ShutdownSample::InertiaSwitch);
        }
        if (hal::gpio::read(k_sdn_aux)) {
            shutdown_samples.set(ShutdownSample::Auxiliary);
        }

        StatusMessage status_message{
            .shutdown_samples = shutdown_samples,
            .ts_activation_desired = ts_activation_desired.has_value(),
            .rtd_activation_desired = rtd_activation_desired.has_value(),
            .apps_calibrated = apps_calibrated,
        };
        hal::can::transmit(config::k_front_can_id, status_message);

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
        hal::can::transmit(config::k_front_can_id, lvs_sample_message_1);

        LvsSampleMessage2 lvs_sample_message_2{
            .dwin_voltage = fuse_voltages[4],
            .aux_1_voltage = fuse_voltages[5],
            .aux_2_voltage = fuse_voltages[6],
        };
        hal::can::transmit(config::k_front_can_id, lvs_sample_message_2);

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
    while (!calibrator.update(s_adc_buffer[7])) {
        sensors[0].update_limits(s_adc_buffer[7]);
        sensors[1].update_limits(s_adc_buffer[8]);
        scheduler.delay_until_ms(k_throttle_period);
    }

    // Notify main task of completed calibration.
    s_main_task.notify_set_bits(0, 1u << 2);

    // Create a default throttle map.
    auto throttle_map = ThrottleMap::create_default();

    while (true) {
        // Read the first sensor, normalise it to a throttle map index, and calculate a travel percentage between the
        // range of [0, 1000].
        // TODO: Look at both sensors.
        // TODO: Current preload.
        const auto normalised = sensors[0].normalise(s_adc_buffer[7]).value_or(0);
        const auto percentage = ThrottleMap::to_percentage(normalised);

        // Calculate a desired throttle (motor current percentage) using the throttle map and a 10% deadzone.
        const std::uint16_t desired_throttle = percentage > 100 ? throttle_map(normalised) : 0;

        ThrottleMessage throttle_message{
            .desired_throttle = desired_throttle,
            .pedal_travel = percentage,
            .raw_1 = s_adc_buffer[7],
            .raw_2 = s_adc_buffer[8],
        };
        hal::can::transmit(config::k_front_can_id, throttle_message);

        scheduler.delay_until_ms(k_throttle_period);
    }
}

void debounce_task(void *) {
    TickType_t last_ts_button_time = 0;
    TickType_t last_rtd_button_time = 0;
    while (true) {
        // Wait for either button to be pressed. Clearing on both entry and exit is important here.
        const auto notification = freertos::notify_wait(0, UINT32_MAX, UINT32_MAX, portMAX_DELAY);

        // Only trigger if the button has been held for a minimum period. A delay is intentional here to ignore other
        // presses in this time.
        vTaskDelay(pdMS_TO_TICKS(k_button_hold_duration));

        // Button triggered if it's still pressed after the delay period and hasn't already been pressed recently.
        const auto current_ticks = xTaskGetTickCount();
        if ((notification & (1u << 0)) != 0 && !hal::gpio::read(k_ts_button) &&
            current_ticks - last_ts_button_time >= pdMS_TO_TICKS(k_button_lockout_duration)) {
            last_ts_button_time = current_ticks;
            s_main_task.notify_set_bits(0, 1u << 0);
        }
        if ((notification & (1u << 1)) != 0 && !hal::gpio::read(k_rtd_button) &&
            current_ticks - last_rtd_button_time >= pdMS_TO_TICKS(k_button_lockout_duration)) {
            last_rtd_button_time = current_ticks;
            s_main_task.notify_set_bits(0, 1u << 1);
        }
    }
}

void led_task(void *) {
    // Configure LED GPIO outputs.
    hal::gpio::configure(k_ts_button_led, hal::gpio::OutputMode::PushPull, hal::gpio::SlewRate::_2M);
    hal::gpio::configure(k_rtd_button_led, hal::gpio::OutputMode::PushPull, hal::gpio::SlewRate::_2M);

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

    s_led_task.notify_give(0);
    while (true) {
        // Wait for a state change.
        freertos::notify_take(0, true, portMAX_DELAY);

        // Set TS button LED.
        DMA1_Channel6->CCR &= ~DMA_CCR_EN;
        if (!s_precharge_state) {
            // Off.
            ts_buffer[0] = 1u << (k_ts_button_led.pin + 16);
            DMA1_Channel6->CNDTR = 1;
        } else if (s_precharge_state == precharge::State::Active) {
            // Solid.
            ts_buffer[0] = 1u << k_ts_button_led.pin;
            DMA1_Channel6->CNDTR = 1;
        } else {
            // Slow flash for standby and fast for everything else.
            const auto count = s_precharge_state == precharge::State::Standby ? 5 : 1;
            for (std::uint32_t i = 0; i < count; i++) {
                ts_buffer[i] = 1u << k_ts_button_led.pin;
                ts_buffer[count + i] = 1u << (k_ts_button_led.pin + 16);
            }
            DMA1_Channel6->CNDTR = count * 2;
        }
        DMA1_Channel6->CCR |= DMA_CCR_EN;

        // Set RTD button LED.
        DMA1_Channel2->CCR &= ~DMA_CCR_EN;
        if (!s_precharge_state || *s_precharge_state != precharge::State::Active) {
            // Off.
            rtd_buffer[0] = 1u << (k_rtd_button_led.pin + 16);
            DMA1_Channel2->CNDTR = 1;
        } else if (s_rear_status && s_rear_status->rtd_prevention_flags.none_set()) {
            // Solid.
            rtd_buffer[0] = 1u << k_rtd_button_led.pin;
            DMA1_Channel2->CNDTR = 1;
        } else {
            // Slow flash to indicate ready to activate, fast for any additional errors set.
            const auto count =
                (s_rear_status && s_rear_status->rtd_prevention_flags.only_set(rear::RtdPreventionFlag::NotRequested))
                    ? 5
                    : 1;
            for (std::uint32_t i = 0; i < count; i++) {
                rtd_buffer[i] = 1u << k_rtd_button_led.pin;
                rtd_buffer[count + i] = 1u << (k_rtd_button_led.pin + 16);
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

        const auto can_stats = hal::can::get_stats();
        hal::swd_printf("CAN status: %s %u/%u %u/%u\n", hal::can::is_online() ? "online" : "offline",
                        can_stats.rx_count, can_stats.lost_rx_count, can_stats.tx_count, can_stats.lost_tx_count);
    }
}

} // namespace

extern "C" void EXTI1_IRQHandler() {
    // Clear pending bit.
    EXTI->PR = EXTI_PR_PR1;

    // Notify debounce task of button press.
    freertos::InterruptYielder interrupt_yielder;
    s_debounce_task.notify_set_bits_isr(0, 1u << 0, interrupt_yielder);
}

extern "C" void EXTI15_10_IRQHandler() {
    // Clear pending bit.
    EXTI->PR = EXTI_PR_PR14;

    // Notify debounce task of button press.
    freertos::InterruptYielder interrupt_yielder;
    s_debounce_task.notify_set_bits_isr(0, 1u << 1, interrupt_yielder);
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
