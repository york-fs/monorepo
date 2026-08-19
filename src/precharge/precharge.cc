#include <can.hh>
#include <config.hh>
#include <freertos.hh>
#include <hal.hh>
#include <node_status.hh>
#include <precharge/can_messages.hh>
#include <precharge/error.hh>
#include <precharge/relay.hh>
#include <precharge/state.hh>
#include <time_tracked.hh>
#include <util.hh>

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

using namespace precharge;

namespace {

/**
 * @brief The maximum time to wait for a new heartbeat message to be received before opening the AIRs and thus
deactivating the TS in milliseconds.
*
* The value chosen here is very specific as it allows ~250 ms for the rear distribution to cut inverter power before the
AIRs open. Note that this is slightly higher than the BMS' delay for hard latching the shutdown circuit so that in the
event of a BMS fault, the precharge notices the event as a shutdown open rather than a deactivation (which should
otherwise be reserved as a less critical error).
 */
constexpr std::uint32_t k_heartbeat_timeout = 250;

/**
 * @brief The extra time to hold the precharge relay after closing the positive AIR in milliseconds.
 */
constexpr std::uint32_t k_precharge_hold_time = 500;

/**
 * @brief State machine task scheduling period in milliseconds.
 */
constexpr std::uint32_t k_sm_period = 10;

/**
 * @brief Hard-coded value of the 3V3 rail powering the STM's ADC in 1 mV resolution.
 */
constexpr std::uint32_t k_mcu_vref = 3300;

/**
 * @brief Bitmask of all values in OutputBit.
 */
constexpr std::uint32_t k_output_mask = 0xfcf7;

/**
 * @brief Port B output pins.
 */
enum class OutputBit : std::uint32_t {
    ActiveStateLed = 0,
    PrechargeStateLed = 1,
    PrechargeCmd = 2,
    DischargeErrorLed = 4,
    AirPosCmd = 5,
    AirNegCmd = 6,
    McuShutdown = 7,
    StandbyStateLed = 10,
    PrecheckStateLed = 11,
    DisconnectErrorLed = 12,
    PrechargeErrorLed = 13,
    AirPosErrorLed = 14,
    AirNegErrorLed = 15,
};

using OutputBits = util::FlagBitset<OutputBit>;

// We want to print the same information over SWD as the status message sent out over CAN.
using SwdData = StatusMessage;

// CAN heartbeat.
TimeTracked<std::monostate> s_heartbeat(k_heartbeat_timeout);

// Tasks.
freertos::Task<256> s_sm_task;
freertos::Task<128> s_swd_task;
freertos::Queue<SwdData, 1> s_swd_queue;

// Input pins.
hal::Gpio s_precharge_sample(hal::GpioPort::A, 1);
hal::Gpio s_tractive_sample(hal::GpioPort::A, 2);
hal::Gpio s_shutdown_sample(hal::GpioPort::A, 8);
hal::Gpio s_precharge_act(hal::GpioPort::A, 9);
hal::Gpio s_air_pos_act(hal::GpioPort::A, 10);
hal::Gpio s_air_neg_act(hal::GpioPort::A, 11);

std::uint16_t convert_voltage(std::uint16_t adc_value) {
    // Convert ADC counts to voltage.
    const auto sampled = (k_mcu_vref * adc_value) >> 12;

    // Convert single-ended conversion value to HV input voltage. The output value of the single-ended conversion op-amp
    // circuit rides around 1.65 volts (VREF/2).
    // TODO: Handle negative values.
    constexpr std::uint32_t ref_value = k_mcu_vref / 2;
    return static_cast<std::uint16_t>(((std::max(sampled, ref_value) - ref_value) * 97) / 400);
}

std::pair<State, ErrorFlags> led_check(std::uint32_t elapsed_ms) {
    return std::make_pair(elapsed_ms > 500 ? State::Precheck : State::LedCheck, ErrorFlags());
}

std::pair<State, ErrorFlags> precheck_standby(std::uint32_t elapsed_ms, std::uint16_t precharge_voltage,
                                              std::uint16_t tractive_voltage) {
    // Check relay actual states. They should all be open with shutdown low (discharge relay closed).
    ErrorFlags error_flags;
    if (s_shutdown_sample.read()) {
        error_flags.set(Error::DischargeOpen);
    }
    if (!s_precharge_act.read()) {
        error_flags.set(Error::PrechargeClosed);
    }
    if (!s_air_pos_act.read()) {
        error_flags.set(Error::AirPosClosed);
    }
    if (!s_air_neg_act.read()) {
        error_flags.set(Error::AirNegClosed);
    }

    // The voltage measured directly after the precharge relay should be zero. Wait for discharge of any residual
    // voltage on the TS side before continuing.
    // TODO: Tune thresholds.
    if (precharge_voltage > 5) {
        error_flags.set(Error::PrecheckVoltage);
    }
    if (tractive_voltage > 5) {
        error_flags.set(Error::WaitingDischarge);
    }
    if (error_flags.any_set()) {
        return std::make_pair(State::Precheck, error_flags);
    }
    if (!s_heartbeat) {
        return std::make_pair(State::Standby, ErrorFlags(Error::WaitingActivation));
    }
    return std::make_pair(State::Precharge, ErrorFlags());
}

std::pair<State, ErrorFlags> precharge(std::uint32_t elapsed_ms, std::uint16_t precharge_voltage,
                                       std::uint16_t tractive_voltage) {
    ErrorFlags error_flags;
    if (!s_heartbeat) {
        error_flags.set(Error::Deactivation);
    }
    if (!s_shutdown_sample.read()) {
        error_flags.set(Error::ShutdownOpen);
    }
    if (s_precharge_act.read()) {
        error_flags.set(Error::PrechargeOpen);
    }
    if (!s_air_pos_act.read()) {
        error_flags.set(Error::AirPosClosed);
    }
    if (s_air_neg_act.read()) {
        error_flags.set(Error::AirNegOpen);
    }

    // Don't continue with bad relays.
    if (error_flags.any_set()) {
        if (error_flags.is_set(Error::Deactivation) || elapsed_ms > 500) {
            return std::make_pair(State::Precheck, error_flags);
        }
        return std::make_pair(State::Precharge, error_flags);
    }

    const auto t = static_cast<float>(elapsed_ms) / 1000.0f;
    const auto R = 1000.0f;
    const auto C = 3300.0e-6f;
    const auto expected_tractive = static_cast<std::uint32_t>(precharge_voltage * (1.0f - std::exp(-t / (R * C))));
    const auto deviation = (expected_tractive > tractive_voltage) ? (expected_tractive - tractive_voltage)
                                                                  : (tractive_voltage - expected_tractive);
    if (deviation > 10) {
        // TODO: Check whether matches against welded discharge curve.
        // TODO: If precharge_voltage == tractive_voltage at t=0 then likely TS+ open circuit.
        return std::make_pair(elapsed_ms > 500 ? State::Precheck : State::Precharge, ErrorFlags(Error::Deviation));
    }
    return std::make_pair(t > (3 * R * C) ? State::PrechargeHold : State::Precharge, ErrorFlags());
}

std::pair<State, ErrorFlags> precharge_hold(std::uint32_t elapsed_ms) {
    ErrorFlags error_flags;
    if (!s_heartbeat) {
        error_flags.set(Error::Deactivation);
    }
    if (!s_shutdown_sample.read()) {
        error_flags.set(Error::ShutdownOpen);
    }
    if (s_precharge_act.read()) {
        error_flags.set(Error::PrechargeOpen);
    }
    if (s_air_pos_act.read()) {
        error_flags.set(Error::AirPosOpen);
    }
    if (s_air_neg_act.read()) {
        error_flags.set(Error::AirNegOpen);
    }

    if (error_flags.any_set()) {
        if (error_flags.is_set(Error::Deactivation) || elapsed_ms > 500) {
            return std::make_pair(State::Precheck, error_flags);
        }
        return std::make_pair(State::PrechargeHold, error_flags);
    }
    return std::make_pair(elapsed_ms >= k_precharge_hold_time ? State::Active : State::PrechargeHold, error_flags);
}

std::pair<State, ErrorFlags> active(std::uint32_t elapsed_ms) {
    ErrorFlags error_flags;
    if (!s_heartbeat) {
        error_flags.set(Error::Deactivation);
    }
    if (!s_shutdown_sample.read()) {
        error_flags.set(Error::ShutdownOpen);
    }
    if (!s_precharge_act.read()) {
        error_flags.set(Error::PrechargeClosed);
    }
    if (s_air_pos_act.read()) {
        error_flags.set(Error::AirPosOpen);
    }
    if (s_air_neg_act.read()) {
        error_flags.set(Error::AirNegOpen);
    }
    return std::make_pair(error_flags.any_set() ? State::Precheck : State::Active, error_flags);
}

std::pair<State, ErrorFlags> advance_state(State state, std::uint32_t elapsed_ms, std::uint16_t precharge_voltage,
                                           std::uint16_t tractive_voltage) {
    switch (state) {
    case State::LedCheck:
        return led_check(elapsed_ms);
    case State::Precharge:
        return precharge(elapsed_ms, precharge_voltage, tractive_voltage);
    case State::PrechargeHold:
        return precharge_hold(elapsed_ms);
    case State::Active:
        return active(elapsed_ms);
    default:
        return precheck_standby(elapsed_ms, precharge_voltage, tractive_voltage);
    }
}

void sm_task(void *) {
    // Initialise CAN on port B.
    can::init(can::Port::B, config::k_can_speed, 1);
    can::listen<HeartbeatMessage, [](const HeartbeatMessage &) {
        s_heartbeat.receive({});
    }>(config::k_precharge_can_id, 0);

    // Initialise periodic node status transmission.
    node_status::init(config::k_precharge_can_id);

    // Enable CAN IRQs.
    hal::irq_enable(CAN1_TX_IRQn, 7);
    hal::irq_enable(CAN1_RX0_IRQn, 6);
    hal::irq_enable(CAN1_SCE_IRQn, 5);

    // Sequence the two HV sampling inputs as well as the STM's internal temperature sensor.
    hal::adc_init(ADC1, 3);
    hal::adc_sequence_channel(ADC1, 1, 1, 0b010u);
    hal::adc_sequence_channel(ADC1, 2, 2, 0b010u);
    hal::adc_sequence_channel(ADC1, 3, 16, 0b111u);

    std::array<std::uint16_t, 3> adc_buffer{};
    hal::adc_init_dma(adc_buffer);

    auto state = State::LedCheck;
    ErrorFlags last_error_flags;
    TickType_t state_epoch_time = xTaskGetTickCount();
    freertos::PeriodScheduler scheduler;
    while (true) {
        // Calculate HV sample inputs.
        const auto precharge_voltage = convert_voltage(adc_buffer[0]);
        const auto tractive_voltage = convert_voltage(adc_buffer[1]);

        // Update heartbeat expiry.
        s_heartbeat.update();

        const auto elapsed_ms = pdTICKS_TO_MS(xTaskGetTickCount() - state_epoch_time);
        const auto [new_state, error_flags] = advance_state(state, elapsed_ms, precharge_voltage, tractive_voltage);
        if (state != new_state) {
            state_epoch_time = xTaskGetTickCount();
            if (state != State::Precheck && state != State::Standby) {
                last_error_flags = error_flags;
            }
            state = new_state;
        }

        // Calculate outputs from current state and error flags.
        OutputBits output_bits;

        // First set the state LEDs.
        if (state == State::Precheck || state == State::LedCheck) {
            output_bits.set(OutputBit::PrecheckStateLed);
        }
        if (state == State::Standby || state == State::LedCheck) {
            output_bits.set(OutputBit::StandbyStateLed);
        }
        if (state == State::Precharge || state == State::PrechargeHold || state == State::LedCheck) {
            output_bits.set(OutputBit::PrechargeStateLed);
        }
        if (state == State::Active || state == State::PrechargeHold || state == State::LedCheck) {
            output_bits.set(OutputBit::ActiveStateLed);
        }

        // Set LED check error LEDs.
        if (state == State::LedCheck) {
            output_bits.set(OutputBit::AirNegErrorLed);
            output_bits.set(OutputBit::AirPosErrorLed);
            output_bits.set(OutputBit::PrechargeErrorLed);
            output_bits.set(OutputBit::DisconnectErrorLed);
            output_bits.set(OutputBit::DischargeErrorLed);
        }

        // Open discharge and close AIR- in precharge and active states.
        if (state == State::Precharge || state == State::PrechargeHold || state == State::Active) {
            output_bits.set(OutputBit::McuShutdown);
            output_bits.set(OutputBit::AirNegCmd);
        }

        // Close AIR+ in precharge to active transition and active states.
        if (state == State::PrechargeHold || state == State::Active) {
            output_bits.set(OutputBit::AirPosCmd);
        }

        // Close precharge relay in precharge and precharge to active transition states.
        if (state == State::Precharge || state == State::PrechargeHold) {
            output_bits.set(OutputBit::PrechargeCmd);
        }

        // Set some error LEDs.
        // TODO: Add more as errors are figured out more.
        if (error_flags.is_set(Error::WaitingDischarge)) {
            output_bits.set(OutputBit::DischargeErrorLed);
        }
        if (error_flags.is_set(Error::PrechargeClosed) || error_flags.is_set(Error::PrechargeOpen)) {
            output_bits.set(OutputBit::PrechargeErrorLed);
        }
        if (error_flags.is_set(Error::AirPosClosed) || error_flags.is_set(Error::AirPosOpen)) {
            output_bits.set(OutputBit::AirPosErrorLed);
        }
        if (error_flags.is_set(Error::AirNegClosed) || error_flags.is_set(Error::AirNegOpen)) {
            output_bits.set(OutputBit::AirNegErrorLed);
        }

        // Set bits all at once.
        GPIOB->ODR = (GPIOB->ODR & ~k_output_mask) | output_bits.value();

        // Build a flag bitset of relay actual states.
        RelayStates relay_states;
        if (!s_shutdown_sample.read()) {
            relay_states.set(RelayState::DischargeClosed);
        }
        if (!s_precharge_act.read()) {
            relay_states.set(RelayState::PrechargeClosed);
        }
        if (!s_air_pos_act.read()) {
            relay_states.set(RelayState::AirPosClosed);
        }
        if (!s_air_neg_act.read()) {
            relay_states.set(RelayState::AirNegClosed);
        }

        // Send status message over CAN.
        ErrorFlags send_flags;
        send_flags.set_all(last_error_flags);
        send_flags.set_all(error_flags);
        StatusMessage status_message{
            .precharge_voltage = precharge_voltage,
            .tractive_voltage = tractive_voltage,
            .error_flags = send_flags,
            .relay_states = relay_states,
            .state = state,
        };
        can::transmit(config::k_precharge_can_id, status_message);

        // Update node status temperature.
        const auto mcu_temp_voltage = (k_mcu_vref * adc_buffer[2]) >> 12;
        node_status::update(mcu_temp_voltage);

        // Update SWD data.
        if constexpr (config::enable_debug_logs()) {
            s_swd_queue.overwrite(status_message);
        }

        // Start next ADC sample and delay until next state machine period.
        hal::adc_start(ADC1);
        scheduler.delay_until_ms(k_sm_period);
    }
}

void swd_task(void *) {
    freertos::PeriodScheduler scheduler;
    while (true) {
        const auto data = *s_swd_queue.receive(portMAX_DELAY);
        scheduler.delay_until_ms(1000);

        hal::swd_printf("--------------------------------\n");
        hal::swd_printf("Uptime: %u\n", freertos::uptime_ms() / 1000);
        hal::swd_printf("State: %u\n", util::to_underlying(data.state));
        hal::swd_printf("Flags: 0x%x\n", data.error_flags.value());
        hal::swd_printf("Precharge: %u\n", data.precharge_voltage);
        hal::swd_printf("Tractive: %u\n", data.tractive_voltage);
        hal::swd_printf("Relay states: 0x%x\n", data.relay_states.value());
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
    s_shutdown_sample.configure(hal::GpioInputMode::Floating);
    s_precharge_act.configure(hal::GpioInputMode::Floating);
    s_air_pos_act.configure(hal::GpioInputMode::Floating);
    s_air_neg_act.configure(hal::GpioInputMode::Floating);

    // Configure outputs on port B.
    for (std::uint32_t pin = 0; pin < 16; pin++) {
        if ((k_output_mask & (1u << pin)) != 0) {
            hal::Gpio(hal::GpioPort::B, pin).configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
        }
    }

    s_sm_task.init(&sm_task, "sm", 2);
    if constexpr (config::enable_debug_logs()) {
        s_swd_queue.init();
        s_swd_task.init(&swd_task, "swd", 0);
    }
    vTaskStartScheduler();
}
