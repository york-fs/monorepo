#include <hal.hh>
#include <util.hh>

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include <algorithm>
#include <array>
#include <cstdint>

// TODO: CAN.
// TODO: Enable watchdog and PVD.

namespace {

/**
 * @brief ADC sampling task scheduling period in milliseconds.
 */
constexpr std::uint32_t k_sample_period = 50;

/**
 * @brief State machine task scheduling period in milliseconds.
 */
constexpr std::uint32_t k_sm_period = 10;

constexpr std::uint32_t k_precharge_hold_time = 500;

constexpr std::uint32_t k_adc_vref = 3300;

enum class Relay {
    AirNegative,
    AirPositive,
    Precharge,
    Discharge,
};

enum class State {
    Standby,
    Precheck,
    Precharge,
    PrechargeHold,
    Active,
};

// Sampled values.
std::uint16_t s_precharge_voltage = 0;
std::uint16_t s_tractive_voltage = 0;
std::int8_t s_mcu_temperature = 0;
SemaphoreHandle_t s_sample_mutex;

// Input pins.
hal::Gpio s_precharge_sample(hal::GpioPort::A, 1);
hal::Gpio s_tractive_sample(hal::GpioPort::A, 2);
hal::Gpio s_shutdown_in(hal::GpioPort::A, 8);
hal::Gpio s_precharge_act(hal::GpioPort::A, 9);
hal::Gpio s_air_pos_act(hal::GpioPort::A, 10);
hal::Gpio s_air_neg_act(hal::GpioPort::A, 11);

consteval auto output_pins() {
    return std::array{0, 1, 2, 4, 5, 6, 7, 10, 11, 12, 13, 14, 15};
}

consteval std::uint32_t output_mask() {
    std::uint32_t mask = 0;
    for (std::uint32_t pin : output_pins()) {
        mask |= 1u << pin;
    }
    return mask;
}

constexpr std::uint32_t led_bit(Relay relay) {
    switch (relay) {
    case Relay::AirNegative:
        return 1u << 15;
    case Relay::AirPositive:
        return 1u << 14;
    case Relay::Precharge:
        return 1u << 13;
    case Relay::Discharge:
        return 1u << 4;
    default:
        return 0;
    }
}

constexpr std::uint32_t led_bit(State state) {
    switch (state) {
    case State::Standby:
        return 1u << 10;
    case State::Precheck:
        return 1u << 11;
    case State::Precharge:
        return 1u << 1;
    case State::PrechargeHold:
        return (1u << 1) | (1u << 0);
    case State::Active:
        return 1u << 0;
    default:
        return 0;
    }
}

std::uint16_t convert_voltage(std::uint16_t adc_value) {
    // Convert ADC counts to voltage.
    const auto sampled = (k_adc_vref * adc_value) >> 12;

    // Convert single-ended conversion value to HV input.
    // TODO: Handle negative values.
    // TODO: Fix reference value to 3V3/2.
    constexpr std::uint32_t ref_value = 1580;
    return static_cast<std::uint16_t>(((std::max(sampled, ref_value) - ref_value) * 97) / 400);
}

void sample_task(void *) {
    // Sequence the two HV sampling inputs as well as the STM's internal temperature sensor.
    hal::adc_init(ADC1, 3);
    hal::adc_sequence_channel(ADC1, 1, 1, 0b010u);
    hal::adc_sequence_channel(ADC1, 2, 2, 0b010u);
    hal::adc_sequence_channel(ADC1, 3, 16, 0b111u);

    std::array<std::uint16_t, 3> adc_buffer{};
    hal::adc_init_dma(adc_buffer);

    TickType_t last_schedule_time = xTaskGetTickCount();
    while (true) {
        // Calculate an approximate temperature using constants from the datasheet.
        const auto temperature_voltage = (k_adc_vref * adc_buffer[2]) >> 12;
        const auto temperature = ((1430 - temperature_voltage) * 10) / 43 + 25;

        // Calculate HV sample inputs.
        const auto precharge_voltage = convert_voltage(adc_buffer[0]);
        const auto tractive_voltage = convert_voltage(adc_buffer[1]);

        // Update global values.
        xSemaphoreTake(s_sample_mutex, portMAX_DELAY);
        s_precharge_voltage = precharge_voltage;
        s_tractive_voltage = tractive_voltage;
        s_mcu_temperature = temperature;
        xSemaphoreGive(s_sample_mutex);

        // Start next ADC sample.
        hal::adc_start(ADC1);
        xTaskDelayUntil(&last_schedule_time, pdMS_TO_TICKS(k_sample_period));
    }
}

// Notes for documentation
// Input voltage range
// CAN input and output
// TSAL integration
// State machine

State update_state(State current_state, std::uint32_t elapsed_time) {
    // TODO: Receive start command from CAN.
    if (current_state == State::Standby) {
        return elapsed_time >= 5000 ? State::Precheck : State::Standby;
    }

    if (current_state == State::Precheck) {
        // Check that the shutdown circuit is fine for activation (discharge relay open).
        // TODO: Check sampled voltages and relay actual states.
        if (s_shutdown_in.read()) {
            return State::Precharge;
        }
        return State::Standby;
    }

    xSemaphoreTake(s_sample_mutex, portMAX_DELAY);
    util::ScopeGuard mutex_release([&] {
        xSemaphoreGive(s_sample_mutex);
    });

    if (current_state == State::Precharge) {
        return s_tractive_voltage > 50 ? State::PrechargeHold : State::Precharge;
    }

    if (current_state == State::PrechargeHold) {
        // Move to active state if hold time expired.
        return elapsed_time >= k_precharge_hold_time ? State::Active : State::PrechargeHold;
    }
    return State::Standby;
}

void sm_task(void *) {
    auto state = State::Standby;
    TickType_t state_epoch_time = xTaskGetTickCount();
    TickType_t last_schedule_time = xTaskGetTickCount();
    while (true) {
        const auto state_elapsed = pdTICKS_TO_MS(xTaskGetTickCount() - state_epoch_time);
        const auto old_state = state;
        state = update_state(state, state_elapsed);
        if (state != old_state) {
            state_epoch_time = xTaskGetTickCount();
        }

        // Calculate new output GPIO states.
        std::uint32_t output_port = GPIOB->ODR & ~output_mask();

        // Keep discharge relay closed in standby state.
        if (state != State::Standby) {
            output_port |= 1u << 7;
        }

        // Close AIR- in precharge and active states.
        if (state == State::Precharge || state == State::PrechargeHold || state == State::Active) {
            output_port |= 1u << 6;
        }

        // Close AIR+ in precharge to active transition and active states.
        if (state == State::PrechargeHold || state == State::Active) {
            output_port |= 1u << 5;
        }

        // Close precharge relay in precharge and precharge to active transition states.
        if (state == State::Precharge || state == State::PrechargeHold) {
            output_port |= 1u << 2;
        }

        // Set the relay LEDs. The AIRs and precharge relays are inverted since low indicates closed relays. The
        // discharge relay is normally closed, so is the inverse of the shutdown signal.
        if (!s_air_neg_act.read()) {
            output_port |= led_bit(Relay::AirNegative);
        }
        if (!s_air_pos_act.read()) {
            output_port |= led_bit(Relay::AirPositive);
        }
        if (!s_precharge_act.read()) {
            output_port |= led_bit(Relay::Precharge);
        }
        if (!s_shutdown_in.read()) {
            output_port |= led_bit(Relay::Discharge);
        }

        // Set the LED for the current state.
        output_port |= led_bit(state);

        // Update all output pins on GPIO port B.
        GPIOB->ODR = output_port;
        xTaskDelayUntil(&last_schedule_time, pdMS_TO_TICKS(k_sm_period));
    }
}

void swd_task(void *) {
    TickType_t last_schedule_time = xTaskGetTickCount();
    while (true) {
        hal::swd_printf("--------------------------------\n");

        xSemaphoreTake(s_sample_mutex, portMAX_DELAY);
        hal::swd_printf("Precharge: %u\n", s_precharge_voltage);
        hal::swd_printf("Tractive: %u\n", s_tractive_voltage);
        hal::swd_printf("MCU temperature: %d\n", s_mcu_temperature);
        xSemaphoreGive(s_sample_mutex);

        xTaskDelayUntil(&last_schedule_time, pdMS_TO_TICKS(1000));
    }
}

} // namespace

void vApplicationIdleHook() {
    hal::enter_sleep_mode(hal::WakeupSource::Interrupt);
}

void app_main() {
    // Configure analog inputs.
    s_precharge_sample.configure(hal::GpioInputMode::Analog);
    s_tractive_sample.configure(hal::GpioInputMode::Analog);

    // Configure digital inputs. These all have external pull-ups/pull-downs.
    s_shutdown_in.configure(hal::GpioInputMode::Floating);
    s_precharge_act.configure(hal::GpioInputMode::Floating);
    s_air_pos_act.configure(hal::GpioInputMode::Floating);
    s_air_neg_act.configure(hal::GpioInputMode::Floating);

    // Configure all simple output pins.
    for (std::uint32_t pin : output_pins()) {
        hal::Gpio(hal::GpioPort::B, pin).configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    }

    s_sample_mutex = xSemaphoreCreateMutex();
    xTaskCreate(&sample_task, "sample", 128, nullptr, 3, nullptr);
    xTaskCreate(&sm_task, "sm", 256, nullptr, 2, nullptr);
    xTaskCreate(&swd_task, "swd", 128, nullptr, 1, nullptr);

    // Start the scheduler which we shouldn't return from.
    vTaskStartScheduler();
}
