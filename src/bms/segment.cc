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

/**
 * @brief MAX14920 AFE version bits.
 */
constexpr std::uint8_t k_afe_version_bits = 0b1010;

enum class AfeStatus {
    Ready,
    NotReady,
    BadSpi,
    Shutdown,
};

// TaskHandle_t s_sleep_task_handle = nullptr;
std::atomic<std::uint16_t> s_voltage_sample_period;
std::atomic<std::uint16_t> s_temperature_sample_period;

std::uint8_t s_i2c_address = 0;
std::array<std::uint8_t, 128> s_i2c_buffer;

std::array<std::uint8_t, 4> s_data;

MessageBufferHandle_t s_i2c_rx_queue;
i2c::StateMachine s_i2c1_sm(i2c::Bus::_1);

// Error tracking.
std::atomic<std::uint32_t> s_i2c_error_count = 0;
std::atomic<std::uint32_t> s_crc_error_count = 0;

// TODO
std::array s_address_pins{
    hal::Gpio(hal::GpioPort::A, 8),
    hal::Gpio(hal::GpioPort::A, 9),
    hal::Gpio(hal::GpioPort::A, 10),
    hal::Gpio(hal::GpioPort::A, 11),
};

// Thermistors connected directly to the STM.
std::array s_mcu_thermistor_enable{
    hal::Gpio(hal::GpioPort::B, 9), hal::Gpio(hal::GpioPort::B, 8), hal::Gpio(hal::GpioPort::B, 12),
    hal::Gpio(hal::GpioPort::A, 1), hal::Gpio(hal::GpioPort::A, 2), hal::Gpio(hal::GpioPort::A, 3),
    hal::Gpio(hal::GpioPort::A, 4),
};

// TODO
hal::Gpio s_adc_cs(hal::GpioPort::A, 5);
hal::Gpio s_afe_cs(hal::GpioPort::A, 7);
hal::Gpio s_afe_en(hal::GpioPort::B, 0);
hal::Gpio s_ref_en(hal::GpioPort::B, 1);
hal::Gpio s_led(hal::GpioPort::B, 5);
hal::Gpio s_sck(hal::GpioPort::B, 13);
hal::Gpio s_miso(hal::GpioPort::B, 14);
hal::Gpio s_mosi(hal::GpioPort::B, 15);
hal::Gpio s_scl_1(hal::GpioPort::B, 6);
hal::Gpio s_sda_1(hal::GpioPort::B, 7);
hal::Gpio s_scl_2(hal::GpioPort::B, 10);
hal::Gpio s_sda_2(hal::GpioPort::B, 11);

[[nodiscard]] AfeStatus afe_command(std::uint16_t balance_bits, std::uint8_t control_bits) {
    std::array<std::uint8_t, 3> data{
        static_cast<std::uint8_t>(balance_bits >> 8u),
        static_cast<std::uint8_t>(balance_bits),
        control_bits,
    };
    if (!hal::spi_transfer(SPI2, s_afe_cs, data, 1)) {
        return AfeStatus::BadSpi;
    }

    // Check version bits are correct.
    if ((data[2] >> 4u) != k_afe_version_bits) {
        return AfeStatus::BadSpi;
    }

    // Check UVLO and thermal shutdown bits.
    if ((data[2] & 0b1101u) != 0u) {
        return AfeStatus::Shutdown;
    }

    // Check ready bit.
    if ((data[2] & 0b10u) != 0u) {
        return AfeStatus::NotReady;
    }
    return AfeStatus::Ready;
}

void sample_voltages_task(void *) {
    static std::uint8_t count = 0;

    TickType_t last_schedule_time = xTaskGetTickCount();
    while (true) {
        s_led.write(!s_led.read());

        // Enter a critical section to make sure data sent over I2C is atomic. Also for CRC.
        taskENTER_CRITICAL();
        count++;
        for (std::size_t i = 0; i < s_data.size(); i++) {
            s_data[i] = count + i;
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

    hal::swd_printf("COMMAND\n");

    s_voltage_sample_period.store(*voltage_sample_period);
    s_temperature_sample_period.store(*temperature_sample_period);
}

void cmd_task(void *) {
    // TODO
    s_i2c_address = 0x40u | ~(GPIOA->IDR >> 8u) & 0xfu;

    hal::swd_printf("Hello world 0x%x\n", s_i2c_address);

    while (true) {
        // if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(k_sleep_timeout)) != 0) {
        //     // Notified to stay awake.
        //     continue;
        // }

        // TODO: Size this array properly so that the message can't be too long.
        std::array<std::uint8_t, 16> rx_bytes{};
        const auto rx_length =
            xMessageBufferReceive(s_i2c_rx_queue, rx_bytes.data(), rx_bytes.size(), pdMS_TO_TICKS(k_sleep_timeout));
        if (rx_length != 0) {
            // Received a master command - stay awake.
            handle_command(std::span(rx_bytes).subspan(0, rx_length));
            continue;
        }

        // MasterCommand master_command{};
        // if (xQueueReceive(s_i2c_rx_queue, &master_command, pdMS_TO_TICKS(k_sleep_timeout)) == pdPASS) {
        //     // Received a master command - stay awake.
        //     handle_command(master_command);
        //     continue;
        // }

        hal::swd_printf("Sleeping\n");

        // Reconfigure SCK and MOSI as regular GPIOs as the peripheral will be deactivated.
        s_sck.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
        s_mosi.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

        // Pull the SPI CS lines high by default (active-low).
        // TODO: Cleanup.
        hal::gpio_set(s_adc_cs, s_afe_cs, s_sck);
        hal::gpio_reset(s_adc_cs);
        hal::gpio_set(s_adc_cs);

        // Turn off LED.
        hal::gpio_reset(s_led);

        // Reconfigure SCL as a regular input for use as an external event and enter stop mode. Also reconfigure SDA to
        // avoid the STM driving it low and upsetting the isolator.
        // for (const auto &pin : {s_scl_1, s_sda_1, s_scl_2, s_sda_2}) {
        //     pin.configure(hal::GpioInputMode::Floating);
        // }

        // Enter stop mode and wait for external event on SCL (PB6).
        // AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI6_PB;
        // EXTI->EMR |= EXTI_EMR_MR6;
        // EXTI->FTSR |= EXTI_FTSR_TR6;
        // SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
        // hal::enter_stop_mode(hal::WakeupSource::Event);
        // SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

        hal::swd_printf("Awoken\n");

        s_voltage_sample_period.store(1000);

        // Woken up from stop mode.
        hal::gpio_set(s_led);

        for (const auto &pin : {s_scl_1, s_sda_1, s_scl_2, s_sda_2}) {
            pin.configure(hal::GpioOutputMode::AlternateOpenDrain, hal::GpioOutputSpeed::Max2);
        }

        // hal::i2c_init(I2C1, s_i2c_address);

        // TRA indicates slave TX or RX.
        // SCL stretched until ADDR cleared and DR filled.
        // TxE set and interrupt generated if ITEVFEN and ITBUFEN set.
        // If TxE set and DR *not* written before the end of the transmission, BTF is set and the interface waits until
        // BTF cleared by reading from SR1 and a write to DR. STOPF set when master sends stop condition and interrupt
        // generated if ITEVFEN set. Cleared by a read of SR1 and a write to CR1.

        // I2C1->CR2 |= I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;

        // I2C1->CR1 |= I2C_CR1_ENGC;
        // I2C1->CR1 |= I2C_CR1_ACK;
    }
}

void swd_task(void *) {
    TickType_t last_schedule_time = xTaskGetTickCount();
    while (true) {
        hal::swd_printf("--------------------------------\n");
        hal::swd_printf("Voltage sample period: %u\n", s_voltage_sample_period.load());
        hal::swd_printf("Temperature sample period: %u\n", s_temperature_sample_period.load());
        vTaskDelayUntil(&last_schedule_time, pdMS_TO_TICKS(1000));
    }
}

void i2c_reinit() {
    hal::enable_irq(I2C1_EV_IRQn, 7);
    hal::enable_irq(I2C1_ER_IRQn, 6);
    if (s_i2c1_sm.state() == i2c::State::Error) {
        s_i2c1_sm.init();
    }
    // Listen on our configured address and the general call address.
    s_i2c1_sm.listen(s_i2c_address, true);
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
        s_i2c1_sm.set_buffer(s_i2c_buffer);
    } else if (state == i2c::State::SlaveRxFinish) {
        if (xMessageBufferSendFromISR(s_i2c_rx_queue, s_i2c_buffer.data(), s_i2c1_sm.head(),
                                      &higher_priority_task_woken) == 0) {
            ++s_i2c_error_count;
        }
    } else if (state == i2c::State::SlaveTx) {
        // Build I2C message.
        util::Stream stream(s_i2c_buffer);
        for (auto byte : s_data) {
            stream.write_byte(byte);
        }
        stream.write_be<std::uint32_t>(hal::crc_compute(stream.bytes()));
        s_i2c1_sm.set_buffer(stream.bytes());
    }

    if (state == i2c::State::Idle || state == i2c::State::SlaveRxFinish || state == i2c::State::Error ||
        state == i2c::State::NoAck) {
        i2c_reinit();
    }
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

extern "C" void I2C1_ER_IRQHandler() {
    s_i2c1_sm.error();
    i2c_reinit();
}

void vApplicationIdleHook() {
    // TODO: Test and disable some clocks.
    // hal::enter_sleep_mode(hal::WakeupSource::Interrupt);
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
    for (const auto &gpio : s_mcu_thermistor_enable) {
        gpio.configure(hal::GpioInputMode::Floating);
    }
    s_ref_en.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_afe_en.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

    // Configure CS pin and enable a pull-up on MISO to avoid floating when no slave is selected.
    s_adc_cs.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_afe_cs.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_miso.configure(hal::GpioInputMode::PullUp);

    s_i2c_rx_queue = xMessageBufferCreate(128);

    // xTaskCreate(&sleep_task, "sleep", 128, nullptr, 4, &s_sleep_task_handle);
    xTaskCreate(&cmd_task, "cmd", 128, nullptr, 4, nullptr);
    xTaskCreate(&sample_voltages_task, "voltages", 128, nullptr, 3, nullptr);
    if constexpr (k_enable_debug_logs) {
        xTaskCreate(&swd_task, "swd", 128, nullptr, 1, nullptr);
    }
    vTaskStartScheduler();
}
