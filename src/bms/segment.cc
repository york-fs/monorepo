#include <hal.hh>
#include <i2c.hh>
#include <max_adc.hh>
#include <stm32f103xb.h>
#include <util.hh>

#include <FreeRTOS.h>
#include <message_buffer.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <optional>

namespace {

/**
 * @brief Whether to enable the SWD debug logging task.
 */
constexpr bool k_enable_debug_logs = false;

/**
 * @brief Sleep timeout in milliseconds.
 */
constexpr std::uint32_t k_sleep_timeout = 2000;

/**
 * @brief Hard-coded value of the on-board precision voltage reference in 100 uV resolution.
 */
constexpr std::uint16_t k_adc_vref = 45000;

/**
 * @brief Cell degraded threshold in ADC counts.
 */
constexpr std::uint16_t k_cell_degraded_threshold = 10;

/**
 * @brief Open cell tap voltage threshold in 100 uV resolution.
 */
constexpr std::uint16_t k_cell_open_threshold = 1000;

/**
 * @brief Thermistor acceptable noise threshold in ADC counts. This value corresponds to around 100 mV.
 */
constexpr std::uint16_t k_thermistor_noise_threshold = 1500;

/**
 * @brief Voltage threshold from the absolute endpoints (0 and Vref) in 100 uV resolution from when to consider a
thermistor as being either open or short circuit.
*/
constexpr std::uint32_t k_thermistor_range_threshold = 3000;

/**
 * @brief Product and die version bits for the MAX14920 AFE.
 */
constexpr std::uint8_t k_afe_version_bits = 0b1010u;

enum class ExpanderRegister : std::uint8_t {
    InputPort0 = 0x00,
    InputPort1 = 0x01,
    OutputPort0 = 0x02,
    OutputPort1 = 0x03,
    PolarityPort0 = 0x04,
    PolarityPort1 = 0x05,
    ConfigurationPort0 = 0x06,
    ConfigurationPort1 = 0x07,
};

// Lock for frontend access.
SemaphoreHandle_t s_afe_mutex;

// I2C communication to master.
i2c::StateMachine s_i2c1_sm(i2c::Bus::_1);
std::uint8_t s_i2c_address = 0;
std::array<std::uint8_t, 128> s_i2c_buffer;
MessageBufferHandle_t s_cmd_queue;

// Command received from master.
std::atomic<bool> s_is_charging;
std::atomic<std::uint16_t> s_balance_bitset;

// Sampled data.
std::array<std::uint16_t, 12> s_correction_table{};
std::array<std::uint16_t, 12> s_voltages{};
std::array<std::int8_t, 23> s_temperatures{};
std::uint16_t s_cell_tap_bitset = 0;
std::uint16_t s_degraded_bitset = 0;
std::uint32_t s_thermistor_bitset = 0;

// Error tracking.
std::atomic<std::uint32_t> s_i2c_error_count = 0;

// I2C slave address configuration pins.
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

// General pins.
hal::Gpio s_afe_en(hal::GpioPort::B, 0);
hal::Gpio s_ref_en(hal::GpioPort::B, 1);
hal::Gpio s_led(hal::GpioPort::B, 5);

// SPI pins for ADC and AFE.
hal::Gpio s_adc_cs(hal::GpioPort::A, 5);
hal::Gpio s_afe_cs(hal::GpioPort::A, 7);
hal::Gpio s_sck(hal::GpioPort::B, 13);
hal::Gpio s_miso(hal::GpioPort::B, 14);
hal::Gpio s_mosi(hal::GpioPort::B, 15);

// I2C pins.
hal::Gpio s_scl_1(hal::GpioPort::B, 6);
hal::Gpio s_sda_1(hal::GpioPort::B, 7);
hal::Gpio s_scl_2(hal::GpioPort::B, 10);
hal::Gpio s_sda_2(hal::GpioPort::B, 11);

[[nodiscard]] bool afe_transfer(std::uint8_t selection, bool allow_leakage) {
    // Make sure bottom control bits are clear.
    selection &= ~0b111u;

    // Enable hold mode if we're selecting a cell.
    const bool is_cell = (selection >> 7) != 0;
    if (is_cell) {
        selection |= 0b100u;
    }

    // Diagnostic mode sinks 10 uA on all CV inputs.
    if (allow_leakage) {
        selection |= 0b10u;
    }

    // Allow balancing if we're not selecting a cell.
    const auto balance = allow_leakage ? s_balance_bitset.load() : 0;

    // TODO: This should use SPI interrupts.
    // TODO: Should we check the individually returned cell bits?
    std::array<std::uint8_t, 3> bytes{
        static_cast<std::uint8_t>(balance >> 8),
        static_cast<std::uint8_t>(balance),
        selection,
    };
    if (!hal::spi_transfer(SPI2, s_afe_cs, bytes, 1)) {
        return false;
    }

    // Check MAX14920 version.
    if ((bytes[2] >> 4) != k_afe_version_bits) {
        return false;
    }

    // Check UVLO, thermal shutdown, and ready bits.
    return (bytes[2] & 0xfu) == 0u;
}

void sample_voltages_raw(std::span<std::optional<std::pair<std::uint16_t, std::uint16_t>>> samples, bool calib) {
    xSemaphoreTake(s_afe_mutex, portMAX_DELAY);

    // Configure the frontend. We want to sample the cells without any leakage paths.
    if (!afe_transfer(calib ? 0u : 0b01011000u, false)) {
        // Failed to configure frontend.
        xSemaphoreGive(s_afe_mutex);
        return;
    }

    // Wait for a sampling period. Use a slightly higher period when charging to allow for recovery from balancing drop.
    vTaskDelay(pdMS_TO_TICKS(calib ? 2000 : s_is_charging ? 250 : 30));

    // Sample all cells in order of most potential to least potential (w.r.t. segment ground).
    for (std::size_t cell = 12; cell > 0; cell--) {
        const auto index = static_cast<std::uint8_t>(cell - 1);

        // Select the cell for output on AOUT in hold mode. The level shift and AOUT settle delay should pass before
        // the first ADC acquisition occurs.
        constexpr std::array<std::uint8_t, 12> index_table{
            0b10000000, 0b11000000, 0b10100000, 0b11100000, 0b10010000, 0b11010000,
            0b10110000, 0b11110000, 0b10001000, 0b11001000, 0b10101000, 0b11101000,
        };
        if (!afe_transfer(index_table[index], false)) {
            // Failed to configure frontend.
            continue;
        }

        // Take successive ADC samples to obtain an average voltage reading.
        samples[index] = max_adc::sample_voltage(SPI2, s_adc_cs, k_adc_vref, 16);
    }

    // Re-enable leakage paths (diagnostic mode and balancing). Diagnostic mode helps to catch disconnected taps.
    static_cast<void>(afe_transfer(0b01011000u, true));
    xSemaphoreGive(s_afe_mutex);
}

void sample_voltages_task(void *) {
    TickType_t last_schedule_time = xTaskGetTickCount();
    while (true) {
        std::array<std::optional<std::pair<std::uint16_t, std::uint16_t>>, 12> samples;
        sample_voltages_raw(samples, false);

        std::uint16_t cell_tap_bitet = 0;
        std::uint16_t degraded_bitset = 0;
        std::array<std::uint16_t, 12> voltages{};
        for (std::size_t index = 0; index < voltages.size(); index++) {
            if (const auto sample = samples[index]) {
                const auto [voltage, adc_range] = *samples[index];

                // Check whether the cell tap is open by checking closeness to the ADC reading endpoints.
                if (voltage < k_cell_open_threshold || voltage > (k_adc_vref - k_cell_open_threshold)) {
                    // Cell tap is bad.
                    continue;
                }

                // Cell tap is connected. Apply parasitic capacitance correction factor.
                // TODO: Do more testing with this.
                cell_tap_bitet |= 1u << index;
                voltages[index] = voltage - s_correction_table[index];

                // Check if the cell tap is degraded (noisy).
                if (adc_range > k_cell_degraded_threshold) {
                    degraded_bitset |= 1u << index;
                }
            }
        }

        // Copy the data in a critical section to make sure the copy is atomic and the data is self-consistent.
        taskENTER_CRITICAL();
        s_cell_tap_bitset = cell_tap_bitet;
        s_degraded_bitset = degraded_bitset;
        std::copy(voltages.begin(), voltages.end(), s_voltages.begin());
        taskEXIT_CRITICAL();

        const auto period = s_is_charging ? 2000 : 100;
        vTaskDelayUntil(&last_schedule_time, pdMS_TO_TICKS(period));
    }
}

hal::I2cStatus set_expander_register(ExpanderRegister reg, std::uint8_t value) {
    std::array data{
        static_cast<std::uint8_t>(reg),
        value,
    };
    if (auto status = hal::i2c_wait_idle(I2C2, 5); status != hal::I2cStatus::Ok) {
        return status;
    }
    if (auto status = hal::i2c_master_write(I2C2, 0x20, data); status != hal::I2cStatus::Ok) {
        return status;
    }
    hal::i2c_stop(I2C2);
    return hal::I2cStatus::Ok;
}

std::optional<std::int8_t> sample_thermistor(std::uint16_t rail_voltage, std::uint8_t index) {
    auto configuration_register = ExpanderRegister::ConfigurationPort0;
    if (index >= s_mcu_thermistor_enable.size() + 8) {
        configuration_register = ExpanderRegister::ConfigurationPort1;
    }

    // Enable the thermistor.
    if (index < s_mcu_thermistor_enable.size()) {
        // Pull the MCU pin high.
        s_mcu_thermistor_enable[index].configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
        hal::gpio_set(s_mcu_thermistor_enable[index]);
    } else {
        // The thermistor order is reversed on port 1 compared to port 0, i.e. the pins go in a clockwise fashion.
        const auto pin_index = index - s_mcu_thermistor_enable.size();
        const auto pin_bit =
            configuration_register == ExpanderRegister::ConfigurationPort0 ? pin_index : 15 - pin_index;
        if (set_expander_register(configuration_register, ~(1u << pin_bit)) != hal::I2cStatus::Ok) {
            // Failed to configure expander.
            return std::nullopt;
        }
    }

    // Allow some settling time.
    hal::delay_us(5);

    // Configure the frontend.
    xSemaphoreTake(s_afe_mutex, portMAX_DELAY);
    if (!afe_transfer(0b00111000u, true)) {
        // Failed to configure frontend.
        xSemaphoreGive(s_afe_mutex);
        return std::nullopt;
    }

    // Sample the voltage on the ADC.
    const auto sample = max_adc::sample_voltage(SPI2, s_adc_cs, k_adc_vref, 8);
    xSemaphoreGive(s_afe_mutex);

    // Disable the thermistor.
    if (index < s_mcu_thermistor_enable.size()) {
        // Reconfigure MCU pin to high impedance.
        s_mcu_thermistor_enable[index].configure(hal::GpioInputMode::Floating);
    } else if (set_expander_register(configuration_register, 0xff) != hal::I2cStatus::Ok) {
        // Failed to configure expander.
        return std::nullopt;
    }

    if (!sample) {
        // Failed to sample ADC.
        return std::nullopt;
    }

    // The min ensures that a bad rail voltage doesn't result in false readings.
    auto [voltage, adc_range] = *sample;
    voltage = std::min(voltage, rail_voltage);

    // Check if the voltage measurement is viable.
    if (voltage < k_thermistor_range_threshold || voltage > (rail_voltage - k_thermistor_range_threshold)) {
        return std::nullopt;
    }

    // Check if the voltage measurement is too noisy.
    if (adc_range > k_thermistor_noise_threshold) {
        return std::nullopt;
    }

    // Thermistor is connected, so we can calculate the temperature.
    // TODO: Use a lookup table/don't use floats here.
    const auto resistance = (rail_voltage * 10000) / voltage - 10000;
    float beta = 1.0f / 3950.0f;
    if (index < 3) {
        // The onboard thermistors have a different beta.
        beta = 1.0f / 3350.0f;
    }
    const float t0 = 1.0f / 298.15f;
    const float temperature = 1.0f / (t0 + beta * std::log(static_cast<float>(resistance) / 10000.0f)) - 273.15f;
    return static_cast<std::int8_t>(temperature);
}

void sample_temperatures_task(void *) {
    TickType_t last_schedule_time = xTaskGetTickCount();
    while (true) {
        std::uint32_t thermistor_bitset = 0;
        std::array<std::int8_t, 23> temperatures{};
        for (std::uint8_t index = 0; index < temperatures.size(); index++) {
            const auto temperature = sample_thermistor(33330, index);
            if (temperature) {
                // Temperature reading is viable.
                thermistor_bitset |= 1u << index;
                temperatures[index] = *temperature;
            }
        }

        // Copy the data in a critical section to make sure the copy is atomic and the data is self-consistent.
        taskENTER_CRITICAL();
        s_thermistor_bitset = thermistor_bitset;
        std::copy(temperatures.begin(), temperatures.end(), s_temperatures.begin());
        taskEXIT_CRITICAL();

        const auto period = s_is_charging ? 2000 : 1000;
        vTaskDelayUntil(&last_schedule_time, pdMS_TO_TICKS(period));
    }
}

void handle_command(std::span<std::uint8_t> bytes) {
    util::Stream stream(bytes);
    const auto mode_byte = stream.read_byte();
    if (!mode_byte) {
        ++s_i2c_error_count;
        return;
    }
    if (*mode_byte != 0xaa && *mode_byte != 0x55) {
        ++s_i2c_error_count;
        return;
    }
    const auto balance_bitset = stream.read_be<std::uint16_t>();
    if (!balance_bitset) {
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
        ++s_i2c_error_count;
        return;
    }

    s_is_charging.store(*mode_byte == 0x55);
    s_balance_bitset.store(*balance_bitset);
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

        // Reconfigure SCK and MOSI as regular GPIOs before going to sleep.
        s_sck.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
        s_mosi.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

        // Pull CS lines high by default (active-low) and put the ADC into shutdown.
        hal::gpio_set(s_adc_cs, s_afe_cs, s_sck);
        hal::gpio_reset(s_adc_cs);
        hal::gpio_set(s_adc_cs);

        // Disable the frontend and reference.
        hal::gpio_reset(s_afe_en, s_ref_en, s_led);

        // Drive all thermistor outputs low to avoid floating power draw.
        s_scl_2.configure(hal::GpioOutputMode::AlternateOpenDrain, hal::GpioOutputSpeed::Max2);
        s_sda_2.configure(hal::GpioOutputMode::AlternateOpenDrain, hal::GpioOutputSpeed::Max2);
        hal::i2c_init(I2C2, std::nullopt);
        static_cast<void>(set_expander_register(ExpanderRegister::OutputPort0, 0x00));
        static_cast<void>(set_expander_register(ExpanderRegister::OutputPort1, 0x00));
        static_cast<void>(set_expander_register(ExpanderRegister::ConfigurationPort0, 0x00));
        static_cast<void>(set_expander_register(ExpanderRegister::ConfigurationPort1, 0x00));
        for (const auto &pin : s_mcu_thermistor_enable) {
            pin.configure(hal::GpioInputMode::PullDown);
        }

        // Reconfigure SCL as a regular input for use as an external event. Also reconfigure SDA to avoid the STM
        // driving it low and upsetting the isolator.
        for (const auto &pin : {s_scl_1, s_sda_1, s_scl_2, s_sda_2}) {
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

        // Woken up from stop mode - enable the frontend and reference.
        hal::gpio_set(s_afe_en, s_ref_en, s_led);

        // Clear saved command.
        s_is_charging.store(false);
        s_balance_bitset.store(0);

        // Listen on I2C.
        for (const auto &pin : {s_scl_1, s_sda_1, s_scl_2, s_sda_2}) {
            pin.configure(hal::GpioOutputMode::AlternateOpenDrain, hal::GpioOutputSpeed::Max2);
        }
        i2c_listen();

        // Initialise I2C2 in master mode for the GPIO expander.
        // TODO: This should use the state machine.
        hal::i2c_init(I2C2, std::nullopt);

        // Wake the ADC.
        hal::gpio_reset(s_sck, s_adc_cs);
        hal::gpio_set(s_adc_cs, s_afe_cs);

        // Configure SCK and MOSI for use with the SPI peripheral.
        s_sck.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max10);
        s_mosi.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max10);

        // Enable SPI2 in master mode at 2 MHz (4x divider).
        hal::spi_init_master(SPI2, SPI_CR1_BR_0);

        // Wait for AFE and reference turn on.
        vTaskDelay(pdMS_TO_TICKS(200));

        // Perform a parasitic capacitance calibration.
        // TODO: More error checking here.
        std::array<std::optional<std::pair<std::uint16_t, std::uint16_t>>, 12> samples;
        sample_voltages_raw(samples, true);
        for (std::size_t i = 0; i < samples.size(); i++) {
            if (const auto sample = samples[i]) {
                s_correction_table[i] = sample->first / 128;
            }
        }

        // Configure expander.
        static_cast<void>(set_expander_register(ExpanderRegister::OutputPort0, 0xff));
        static_cast<void>(set_expander_register(ExpanderRegister::OutputPort1, 0xff));
        static_cast<void>(set_expander_register(ExpanderRegister::PolarityPort0, 0x00));
        static_cast<void>(set_expander_register(ExpanderRegister::PolarityPort1, 0x00));
        static_cast<void>(set_expander_register(ExpanderRegister::ConfigurationPort0, 0xff));
        static_cast<void>(set_expander_register(ExpanderRegister::ConfigurationPort1, 0xff));
    }
}

void swd_task(void *) {
    TickType_t last_schedule_time = xTaskGetTickCount();
    while (true) {
        hal::swd_printf("--------------------------------\n");
        hal::swd_printf("Mode: %s\n", s_is_charging ? "charging" : "discharging");
        hal::swd_printf("Balance: 0x%x\n", s_balance_bitset.load());
        hal::swd_printf("Balance temperatures: [%d, %d, %d]\n", s_temperatures[0], s_temperatures[1],
                        s_temperatures[2]);
        hal::swd_printf("I2C error count: %u\n", s_i2c_error_count.load());
        hal::swd_printf("C[1, 6]:  [%u, %u, %u, %u, %u, %u]\n", s_correction_table[0], s_correction_table[1],
                        s_correction_table[2], s_correction_table[3], s_correction_table[4], s_correction_table[5]);
        hal::swd_printf("C[7, 12]: [%u, %u, %u, %u, %u, %u]\n", s_correction_table[6], s_correction_table[7],
                        s_correction_table[8], s_correction_table[9], s_correction_table[10], s_correction_table[11]);
        hal::swd_printf("V[1, 6]:  [%u, %u, %u, %u, %u, %u]\n", s_voltages[0], s_voltages[1], s_voltages[2],
                        s_voltages[3], s_voltages[4], s_voltages[5]);
        hal::swd_printf("V[7, 12]: [%u, %u, %u, %u, %u, %u]\n", s_voltages[6], s_voltages[7], s_voltages[8],
                        s_voltages[9], s_voltages[10], s_voltages[11]);
        hal::swd_printf("T[1, 10]: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d]\n", s_temperatures[3], s_temperatures[4],
                        s_temperatures[5], s_temperatures[6], s_temperatures[7], s_temperatures[8], s_temperatures[9],
                        s_temperatures[10], s_temperatures[11], s_temperatures[12]);
        hal::swd_printf("T[11, 20]: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d]\n", s_temperatures[13], s_temperatures[14],
                        s_temperatures[15], s_temperatures[16], s_temperatures[17], s_temperatures[18],
                        s_temperatures[19], s_temperatures[20], s_temperatures[21], s_temperatures[22]);
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
        stream.write_be(s_i2c_error_count.load());
        stream.write_be(s_thermistor_bitset);
        stream.write_be(s_cell_tap_bitset);
        stream.write_be(s_degraded_bitset);
        for (std::uint16_t voltage : s_voltages) {
            stream.write_be(voltage);
        }
        for (std::int8_t temperature : s_temperatures) {
            stream.write_byte(temperature);
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
    s_afe_en.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_ref_en.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

    // Configure SPI chip select pins and enable a pull-up on MISO to avoid floating when no slave is selected.
    s_adc_cs.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_afe_cs.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_miso.configure(hal::GpioInputMode::PullUp);

    s_afe_mutex = xSemaphoreCreateMutex();
    s_cmd_queue = xMessageBufferCreate(128);

    xTaskCreate(&cmd_task, "cmd", 128, nullptr, 4, nullptr);
    xTaskCreate(&sample_voltages_task, "voltages", 128, nullptr, 3, nullptr);
    xTaskCreate(&sample_temperatures_task, "temperatures", 128, nullptr, 2, nullptr);
    if constexpr (k_enable_debug_logs) {
        xTaskCreate(&swd_task, "swd", 128, nullptr, 1, nullptr);
    }
    vTaskStartScheduler();
}
