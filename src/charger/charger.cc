#include <can.hh>
#include <charger/can_messages.hh>
#include <charger/error.hh>
#include <config.hh>
#include <hal.hh>

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include <array>
#include <cstdint>

using namespace charger;

namespace {

/**
 * @brief Whether to enable the SWD debug logging task.
 */
constexpr bool k_enable_debug_logs = true;

/**
 * @brief The maximum allowed value for the current setpoint in 1 mA resolution.
 */
constexpr std::uint32_t k_current_limit = 3000;

/**
 * @brief The maximum input voltage allowed on the charge power rail in 1 mV resolution.
 */
constexpr std::uint32_t k_max_voltage = 58000;

/**
 * @brief Voltage setpoint fudge factor to account for diode drop and ohmic loss in the main current carrying
 * conductors. This is quite sad but is necessary with hardware revision A since the MCU doesn't sample the MOSFET drop.
 */
constexpr std::uint32_t k_voltage_fudge = 800;

constexpr std::uint32_t k_cv_set_ratio = 205;

/**
 * @brief Calculated maximum drop allowable across the MOSFET in terms of the CV set point. This is effectively the
 * maximum difference allowed between the charge voltage and the current battery voltage. In revision A, it is
 * around 16.4 volts, which is a reasonable limit since the MOSFET can't drop much power anyway.
 */
constexpr std::uint32_t k_maximum_drop = (1024 * 3300) / k_cv_set_ratio;

struct SwdData {
    ErrorFlags error_flags;
    std::uint16_t charge_voltage;
    std::uint16_t target_current;
    std::uint16_t target_voltage;
    bool enabled;
};

// Control loop.
QueueHandle_t s_control_queue;
QueueHandle_t s_swd_queue;

hal::Gpio s_vcc_sense(hal::GpioPort::A, 2);
hal::Gpio s_cc_set(hal::GpioPort::A, 8);
hal::Gpio s_cv_set(hal::GpioPort::A, 9);
hal::Gpio s_enable(hal::GpioPort::A, 12);
hal::Gpio s_led(hal::GpioPort::B, 6);

[[nodiscard]] ErrorFlags set_current(std::uint32_t current) {
    ErrorFlags error_flags;
    if (current > k_current_limit) {
        error_flags.set(Error::Overcurrent);
    }

    // Multiply by CC_SET divider ratio.
    current *= 31;

    // Multiply by current shunt ratio.
    current *= 25;

    if (!error_flags.any_set()) {
        // Convert to PWM duty cycle.
        TIM1->CCR1 = util::clamp(((current / 3300) * 1024) / 1000, 0, 1024);
    } else {
        // Default to zero current.
        TIM1->CCR1 = 0;
    }
    return error_flags;
}

[[nodiscard]] ErrorFlags set_voltage(std::uint32_t charge_voltage, std::uint32_t target_voltage) {
    // Check for overvoltage on power rail.
    ErrorFlags error_flags;
    if (charge_voltage > k_max_voltage) {
        error_flags.set(Error::RailOvervoltage);
    }

    // Factor in any drop into the effective available charge voltage.
    charge_voltage -= std::min(charge_voltage, k_voltage_fudge);

    // Make sure the desired target voltage is possible.
    if (target_voltage > charge_voltage) {
        error_flags.set(Error::RailUndervoltage);
    }

    // Calculate the desired minimum drop across the MOSFET, or put another way, the maximum battery voltage.
    const auto drop_voltage = charge_voltage - target_voltage;
    if (drop_voltage > k_maximum_drop) {
        // The desired drop would be above the maximum setpoint.
        error_flags.is_set(Error::DropOvervoltage);
    }

    if (!error_flags.any_set()) {
        // Convert to PWM duty cycle.
        TIM1->CCR2 = util::clamp((drop_voltage * k_cv_set_ratio) / 3300, 0, 1024);
    } else {
        // Default to max value.
        TIM1->CCR2 = 1024;
    }
    return error_flags;
}

void control_task(void *) {
    // Initialise CAN on port B.
    can::init(can::Port::B, config::k_can_speed, 3, 8);
    can::listen<ControlMessage, [](const ControlMessage &control) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        xQueueSendToBackFromISR(s_control_queue, &control, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }>(config::k_charger_can_id, 0, 0);

    // Enable CAN IRQs.
    hal::enable_irq(CAN1_TX_IRQn, 7);
    hal::enable_irq(CAN1_RX0_IRQn, 6);
    hal::enable_irq(CAN1_SCE_IRQn, 5);

    // Sequence the charge voltage sampling and the STM's internal temperature sensor.
    hal::adc_init(ADC1, 2);
    hal::adc_sequence_channel(ADC1, 1, 2, 0b010u);
    hal::adc_sequence_channel(ADC1, 2, 16, 0b111u);

    std::array<std::uint16_t, 2> adc_buffer;
    hal::adc_init_dma(adc_buffer);

    // Enable automatic continuous ADC sampling.
    ADC1->CR2 |= ADC_CR2_CONT;
    hal::adc_start(ADC1);

    // Enable timer for CC and CV output via PWM.
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->PSC = 0;
    TIM1->ARR = 1024 - 1;
    TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;
    TIM1->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;
    TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC1E;
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1 |= TIM_CR1_ARPE;
    TIM1->CR1 |= TIM_CR1_CEN;

    TickType_t last_receive_time = 0;
    std::uint16_t charge_voltage = 0;
    std::uint16_t target_current = 0;
    std::uint16_t target_voltage = 0;
    bool enable_requested = false;
    while (true) {
        // Run when a new command comes, or every 200 ms.
        ControlMessage message;
        if (xQueueReceive(s_control_queue, &message, pdMS_TO_TICKS(200)) == pdPASS) {
            last_receive_time = xTaskGetTickCount();
            target_current = message.target_current;
            target_voltage = message.target_voltage;
            enable_requested = message.enable;
        }

        // Calculate charge rail voltage.
        // TODO: Calculate MCU temperature.
        charge_voltage = ((static_cast<std::uint32_t>(adc_buffer[0]) * 3300) >> 12) * 18;

        // Build new error flags.
        ErrorFlags error_flags;
        if (!enable_requested) {
            error_flags.set(Error::Disabled);
        }
        if (!can::is_online()) {
            error_flags.set(Error::CanOffline);
        }

        // Set current target.
        error_flags.set_all(set_current(target_current));

        // Set voltage limit.
        error_flags.set_all(set_voltage(charge_voltage, target_voltage));

        // Don't enable if we haven't had a control command in a while.
        if (xTaskGetTickCount() - last_receive_time >= pdMS_TO_TICKS(1000)) {
            error_flags.set(Error::CommunicationTimeout);
        }

        // Update enable pin. The pin is inverted.
        const bool enable = !error_flags.any_set();
        s_enable.write(!enable);
        s_led.write(enable);

        // Send status message over CAN.
        StatusMessage status_message{
            .error_flags = error_flags,
            .charge_voltage = charge_voltage,
            .enabled = enable,
        };
        can::transmit(config::k_charger_can_id, status_message);

        // Update SWD data.
        SwdData swd_data{
            .error_flags = error_flags,
            .charge_voltage = charge_voltage,
            .target_current = target_current,
            .target_voltage = target_voltage,
            .enabled = enable,
        };
        xQueueOverwrite(s_swd_queue, &swd_data);
    }
}

void swd_task(void *) {
    TickType_t last_schedule_time = xTaskGetTickCount();
    while (true) {
        SwdData data;
        xQueueReceive(s_swd_queue, &data, portMAX_DELAY);
        xTaskDelayUntil(&last_schedule_time, pdMS_TO_TICKS(200));

        hal::swd_printf("-------------------------\n");
        hal::swd_printf("Enabled: %s\n", data.enabled ? "yes" : "no");
        hal::swd_printf("CAN online: %s\n", can::is_online() ? "yes" : "no");
        hal::swd_printf("Error flags: 0x%x\n", data.error_flags.value());
        hal::swd_printf("Charge voltage: %u mV\n", data.charge_voltage);
        hal::swd_printf("Target current: %u mA\n", data.target_current);
        hal::swd_printf("Target voltage: %u mV\n", data.target_voltage);
    }
}

} // namespace

void app_main() {
    s_vcc_sense.configure(hal::GpioInputMode::Analog);
    s_cc_set.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max50);
    s_cv_set.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max50);
    s_enable.configure(hal::GpioOutputMode::OpenDrain, hal::GpioOutputSpeed::Max2);
    s_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

    // Default open-drain enable pin to high.
    hal::gpio_set(s_enable);

    s_control_queue = xQueueCreate(8, sizeof(ControlMessage));
    s_swd_queue = xQueueCreate(1, sizeof(SwdData));

    xTaskCreate(&control_task, "control", 128, nullptr, 4, nullptr);
    if constexpr (k_enable_debug_logs) {
        xTaskCreate(&swd_task, "swd", 128, nullptr, 1, nullptr);
    }
    vTaskStartScheduler();
}
