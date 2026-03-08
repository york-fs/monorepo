#include <hal.hh>
#include <i2c.hh>
#include <stm32f103xb.h>
#include <util.hh>

#include <FreeRTOS.h>
#include <message_buffer.h>
#include <queue.h>
#include <task.h>

#include <array>
#include <atomic>
#include <cstdint>

namespace {

/**
 * @brief Whether to enable the SWD debug logging task.
 */
constexpr bool k_enable_debug_logs = true;

/**
 * @brief Sleep timeout in milliseconds.
 */
constexpr std::uint32_t k_sleep_timeout = 2000;

// I2C communication to master.
i2c::StateMachine s_i2c1_sm(i2c::Bus::_1);
std::uint8_t s_i2c_address = 0;
std::array<std::uint8_t, 128> s_i2c_buffer;
MessageBufferHandle_t s_cmd_queue;

// Sampling periods received from master.
std::atomic<std::uint16_t> s_voltage_sample_period;
std::atomic<std::uint16_t> s_temperature_sample_period;

// Sampled data.
std::array<std::uint16_t, 12> s_voltages{};

// Error tracking.
std::atomic<std::uint32_t> s_i2c_error_count = 0;
std::atomic<std::uint32_t> s_crc_error_count = 0;

std::array s_address_pins{
    hal::Gpio(hal::GpioPort::A, 8),
    hal::Gpio(hal::GpioPort::A, 9),
    hal::Gpio(hal::GpioPort::A, 10),
    hal::Gpio(hal::GpioPort::A, 11),
};

hal::Gpio s_led(hal::GpioPort::B, 5);
hal::Gpio s_scl_1(hal::GpioPort::B, 6);
hal::Gpio s_sda_1(hal::GpioPort::B, 7);

void sample_voltages_task(void *) {
    TickType_t last_schedule_time = xTaskGetTickCount();
    while (true) {
        // Enter a critical section to make sure data sent over I2C is atomic. Also for CRC.
        taskENTER_CRITICAL();
        for (std::size_t i = 0; i < s_voltages.size(); i++) {
            s_voltages[i] = 1000 + i;
        }
        taskEXIT_CRITICAL();

        const auto period = util::clamp(s_voltage_sample_period.load(), 200, 1000);
        vTaskDelayUntil(&last_schedule_time, pdMS_TO_TICKS(period));
    }
}

void handle_command(std::span<std::uint8_t> bytes) {
    util::Stream stream(bytes);
    const auto voltage_sample_period = stream.read_be<std::uint16_t>();
    if (!voltage_sample_period) {
        ++s_i2c_error_count;
        return;
    }
    const auto temperature_sample_period = stream.read_be<std::uint16_t>();
    if (!temperature_sample_period) {
        ++s_i2c_error_count;
        return;
    }
    const auto expected_crc = stream.read_be<std::uint32_t>();
    if (!expected_crc) {
        ++s_i2c_error_count;
        return;
    }

    // Compute and compare the actual CRC.
    taskENTER_CRITICAL();
    const auto crc = hal::crc_compute(bytes.subspan(0, bytes.size() - 4));
    taskEXIT_CRITICAL();
    if (expected_crc != crc) {
        ++s_crc_error_count;
        return;
    }

    s_voltage_sample_period.store(*voltage_sample_period);
    s_temperature_sample_period.store(*temperature_sample_period);
}

void i2c_listen() {
    hal::enable_irq(I2C1_EV_IRQn, 6);
    hal::enable_irq(I2C1_ER_IRQn, 6);
    if (s_i2c1_sm.state() == i2c::State::Error) {
        ++s_i2c_error_count;
        s_i2c1_sm.init();
    }
    // Listen on our configured address and the general call address.
    s_i2c1_sm.listen(s_i2c_address, true);
}

void cmd_task(void *) {
    s_i2c_address = 0x40u | ~(GPIOA->IDR >> 8u) & 0xfu;

    s_i2c1_sm.init();
    while (true) {
        std::array<std::uint8_t, 16> cmd_bytes{};
        const auto cmd_length =
            xMessageBufferReceive(s_cmd_queue, cmd_bytes.data(), cmd_bytes.size(), pdMS_TO_TICKS(k_sleep_timeout));
        if (cmd_length != 0) {
            // Received a master command - handle it and stay awake.
            handle_command(std::span(cmd_bytes).subspan(0, cmd_length));
            continue;
        }

        // Turn off the LED.
        hal::gpio_reset(s_led);

        // Reconfigure SCL as a regular input for use as an external event. Also reconfigure SDA to avoid the STM
        // driving it low and upsetting the isolator.
        for (const auto &pin : {s_scl_1, s_sda_1}) {
            pin.configure(hal::GpioInputMode::Floating);
        }

        // Setup external event on SCL (PB6).
        // TODO: Make a HAL function for this.
        AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI6_PB;
        EXTI->EMR |= EXTI_EMR_MR6;
        EXTI->FTSR |= EXTI_FTSR_TR6;

        // Enter stop mode. Disable SysTick to avoid an STM errata.
        SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
        hal::enter_stop_mode(hal::WakeupSource::Event);
        SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

        // Woken up from stop mode.
        hal::gpio_set(s_led);

        // Re-enable listening on I2C.
        for (const auto &pin : {s_scl_1, s_sda_1}) {
            pin.configure(hal::GpioOutputMode::AlternateOpenDrain, hal::GpioOutputSpeed::Max2);
        }
        i2c_listen();
    }
}

void swd_task(void *) {
    TickType_t last_schedule_time = xTaskGetTickCount();
    while (true) {
        hal::swd_printf("--------------------------------\n");
        hal::swd_printf("Voltage sample period: %u\n", s_voltage_sample_period.load());
        hal::swd_printf("Temperature sample period: %u\n", s_temperature_sample_period.load());
        hal::swd_printf("I2C stats: %u %u\n", s_i2c_error_count.load(), s_crc_error_count.load());
        vTaskDelayUntil(&last_schedule_time, pdMS_TO_TICKS(1000));
    }
}

} // namespace

extern "C" void I2C1_EV_IRQHandler() {
    if (!s_i2c1_sm.event()) {
        // State not changed.
        return;
    }

    BaseType_t higher_priority_task_woken = pdFALSE;
    const auto state = s_i2c1_sm.state();
    if (state == i2c::State::SlaveRx) {
        s_i2c1_sm.set_buffer(std::span(s_i2c_buffer).subspan(0, 16));
    } else if (state == i2c::State::SlaveRxFinish) {
        if (xMessageBufferSendFromISR(s_cmd_queue, s_i2c_buffer.data(), s_i2c1_sm.head(),
                                      &higher_priority_task_woken) == 0) {
            ++s_i2c_error_count;
        }
    } else if (state == i2c::State::SlaveTx) {
        // Build I2C message.
        util::Stream stream(s_i2c_buffer);
        for (std::uint16_t voltage : s_voltages) {
            stream.write_be(voltage);
        }
        stream.write_be<std::uint32_t>(hal::crc_compute(stream.bytes()));
        s_i2c1_sm.set_buffer(stream.bytes());
    }

    if (state == i2c::State::Idle || state == i2c::State::SlaveRxFinish || state == i2c::State::Error ||
        state == i2c::State::NoAck) {
        i2c_listen();
    }
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

extern "C" void I2C1_ER_IRQHandler() {
    s_i2c1_sm.error();
    i2c_listen();
}

void vApplicationIdleHook() {
    // TODO: Test power usage and disable some clocks.
    hal::enter_sleep_mode(hal::WakeupSource::Interrupt);
}

bool hal_low_power() {
    // Use the 8 MHz internal clock.
    return true;
}

void app_main() {
    // Configure general GPIOs.
    for (const auto &gpio : s_address_pins) {
        gpio.configure(hal::GpioInputMode::PullUp);
    }
    s_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

    s_cmd_queue = xMessageBufferCreate(128);
    xTaskCreate(&cmd_task, "cmd", 128, nullptr, 4, nullptr);
    xTaskCreate(&sample_voltages_task, "voltages", 128, nullptr, 3, nullptr);
    if constexpr (k_enable_debug_logs) {
        xTaskCreate(&swd_task, "swd", 128, nullptr, 1, nullptr);
    }
    vTaskStartScheduler();
}
