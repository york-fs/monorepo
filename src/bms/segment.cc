#include <freertos.hh>
#include <hal.hh>
#include <i2c.hh>
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
constexpr bool k_enable_debug_logs = true;

/**
 * @brief Sleep timeout in milliseconds.
 */
constexpr std::uint32_t k_sleep_timeout = 2000;

/**
 * @brief Hard-coded value of the on-board precision voltage reference in 100 uV resolution.
 */
constexpr std::uint16_t k_adc_vref = 45000;

/**
 * @brief Hard-coded value of the 3V3 rail powering the STM's ADC in 1 mV resolution.
 */
constexpr std::uint32_t k_mcu_vref = 3333;

/**
 * @brief Cell degraded threshold in ADC counts.
 */
constexpr std::uint16_t k_cell_degraded_threshold = 10;

/**
 * @brief Open cell tap voltage threshold in 100 uV resolution.
 */
constexpr std::uint16_t k_cell_open_threshold = 1000;

/**
 * @brief Voltage threshold from the absolute endpoints (0 and Vref) in 1 mV resolution from when to consider a
thermistor as being either open or short circuit.
*/
constexpr std::uint32_t k_thermistor_range_threshold = 100;

/**
 * @brief Product and die version bits for the MAX14921 AFE.
 */
constexpr std::uint8_t k_afe_version_bits = 0b0010u;

/**
 * @brief Maximum connected cell count.
 */
constexpr std::uint32_t k_cell_count = 16;

/**
 * @brief Maximum connected thermistor count, including the on-board ones.
 */
constexpr std::uint32_t k_thermistor_count = 24;

/**
 * @brief Thermistor MUX mapping array.
 */
constexpr std::array<std::array<std::uint32_t, 3>, 8> k_thermistor_mapping{{
    {2, 13, 21},
    {1, 15, 22},
    {0, 10, 23},
    {3, 12, 20},
    {6, 11, 18},
    {5, 9, 16},
    {7, 14, 19},
    {4, 8, 17},
}};
static_assert(k_thermistor_mapping.size() * k_thermistor_mapping[0].size() == k_thermistor_count);

// I2C communication to master.
i2c::StateMachine s_i2c_sm(i2c::Bus::_1);
std::uint8_t s_i2c_address = 0;
std::array<std::uint8_t, 128> s_i2c_buffer;
freertos::MessageBuffer<128> s_cmd_queue;

// Command received from master.
std::atomic<bool> s_is_charging;
std::atomic<std::uint16_t> s_balance_bitset;

// Sampled data.
std::array<std::uint16_t, k_cell_count> s_correction_table{};
std::array<std::uint16_t, k_cell_count> s_voltages{};
std::array<std::int8_t, k_thermistor_count> s_temperatures{};
std::uint16_t s_cell_tap_bitset = 0;
std::uint16_t s_degraded_bitset = 0;
std::uint16_t s_rail_voltage = 0;
std::uint16_t s_ref_voltage = 0;
std::uint32_t s_thermistor_bitset = 0;

// Error tracking.
std::atomic<std::uint32_t> s_i2c_error_count = 0;

// Tasks.
freertos::Task<256> s_cmd_task;
freertos::Task<128> s_sample_voltages_task;
freertos::Task<128> s_sample_temperatures_task;
freertos::Task<128> s_swd_task;

// I2C slave address configuration pins.
std::array s_address_pins{
    hal::Gpio(hal::GpioPort::A, 8),
    hal::Gpio(hal::GpioPort::A, 9),
    hal::Gpio(hal::GpioPort::A, 10),
    hal::Gpio(hal::GpioPort::A, 11),
};

// Thermistor control and input pins.
hal::Gpio s_rail_sample(hal::GpioPort::A, 1);
std::array s_mux_sample{
    hal::Gpio(hal::GpioPort::A, 2),
    hal::Gpio(hal::GpioPort::A, 3),
    hal::Gpio(hal::GpioPort::A, 4),
};
hal::Gpio s_mux_en(hal::GpioPort::B, 10);
std::array s_mux_control{
    hal::Gpio(hal::GpioPort::B, 11),
    hal::Gpio(hal::GpioPort::B, 9),
    hal::Gpio(hal::GpioPort::B, 8),
};

// General pins.
hal::Gpio s_afe_en(hal::GpioPort::B, 0);
hal::Gpio s_ref_en(hal::GpioPort::B, 1);
hal::Gpio s_led(hal::GpioPort::B, 5);
hal::Gpio s_ref_sample(hal::GpioPort::A, 7);

// SPI pins for ADC and AFE.
hal::Gpio s_adc_cs(hal::GpioPort::A, 5);
hal::Gpio s_afe_cs(hal::GpioPort::A, 6);
hal::Gpio s_sck(hal::GpioPort::B, 13);
hal::Gpio s_miso(hal::GpioPort::B, 14);
hal::Gpio s_mosi(hal::GpioPort::B, 15);

// I2C pins.
hal::Gpio s_scl(hal::GpioPort::B, 6);
hal::Gpio s_sda(hal::GpioPort::B, 7);

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

    // Check AFE version.
    if ((bytes[2] >> 4) != k_afe_version_bits) {
        return false;
    }

    // Check UVLO, thermal shutdown, and ready bits.
    return (bytes[2] & 0xfu) == 0u;
}

std::optional<std::uint16_t> adc_sample_raw() {
    // Trigger conversion.
    hal::gpio_reset(s_adc_cs);
    hal::gpio_set(s_adc_cs);

    // Wait maximum conversion time.
    // TODO: This should be done with interrupts like in the master firmware.
    hal::delay_us(3);

    // Read value over SPI.
    std::array<std::uint8_t, 2> bytes{};
    if (!hal::spi_transfer(SPI2, s_adc_cs, bytes, 2)) {
        return std::nullopt;
    }

    // Return assembled value.
    return (static_cast<std::uint16_t>(bytes[0]) << 8u) | bytes[1];
}

std::optional<std::pair<std::uint16_t, std::uint16_t>> adc_sample_voltage(std::uint32_t sample_count) {
    auto min_value = std::numeric_limits<std::uint16_t>::max();
    auto max_value = std::numeric_limits<std::uint16_t>::min();
    std::uint32_t sum = 0;
    for (std::uint32_t i = 0; i < sample_count; i++) {
        const auto value = adc_sample_raw();
        if (!value) {
            return std::nullopt;
        }
        min_value = std::min(min_value, *value);
        max_value = std::max(max_value, *value);
        sum += *value;
    }

    // Calculate the average in ADC counts and convert that to a voltage, taking care to avoid truncation bias.
    const auto average = (sum + (sample_count / 2)) / sample_count;
    const auto voltage = (average * k_adc_vref + (1u << 15)) >> 16u;
    return std::make_pair(voltage, max_value - min_value);
}

void sample_voltages_raw(std::span<std::optional<std::pair<std::uint16_t, std::uint16_t>>> samples, bool calib) {
    // Configure the frontend. We want to sample the cells without any leakage paths.
    if (!afe_transfer(calib ? 0u : 0b01011000u, false)) {
        // Failed to configure frontend.
        return;
    }

    // Wait for a sampling period. Use a slightly higher period when charging to allow for recovery from balancing drop.
    vTaskDelay(pdMS_TO_TICKS(calib ? 1000 : s_is_charging ? 250 : 30));

    // Enter hold mode early for charge injection calibration.
    if (calib && !afe_transfer(0b10000000, false)) {
        // Failed to configure frontend.
        return;
    }

    // Sample all cells in order of most potential to least potential (w.r.t. segment ground).
    for (std::size_t cell = k_cell_count; cell > 0; cell--) {
        const auto index = static_cast<std::uint8_t>(cell - 1);

        // Select the cell for output on AOUT in hold mode. The level shift and AOUT settle delay should pass before
        // the first ADC acquisition occurs.
        constexpr std::array<std::uint8_t, k_cell_count> index_table{
            0b10000000, 0b11000000, 0b10100000, 0b11100000, 0b10010000, 0b11010000, 0b10110000, 0b11110000,
            0b10001000, 0b11001000, 0b10101000, 0b11101000, 0b10011000, 0b11011000, 0b10111000, 0b11111000,
        };
        if (!afe_transfer(index_table[index], false)) {
            // Failed to configure frontend.
            continue;
        }

        // Discard the first ADC reading.
        if (calib) {
            adc_sample_raw();
        }

        // Take successive ADC samples to obtain an average voltage reading.
        samples[index] = adc_sample_voltage(16);
    }

    // Re-enable leakage paths (diagnostic mode and balancing). Diagnostic mode helps to catch disconnected taps.
    static_cast<void>(afe_transfer(0b01011000u, true));
}

void sample_voltages_task(void *) {
    TickType_t last_schedule_time = xTaskGetTickCount();
    while (true) {
        std::array<std::optional<std::pair<std::uint16_t, std::uint16_t>>, k_cell_count> samples;
        sample_voltages_raw(samples, false);

        std::uint16_t cell_tap_bitet = 0;
        std::uint16_t degraded_bitset = 0;
        std::array<std::uint16_t, k_cell_count> voltages{};
        for (std::size_t index = 0; index < voltages.size(); index++) {
            if (const auto sample = samples[index]) {
                const auto [voltage, adc_range] = *samples[index];

                // Check whether the cell tap is open by checking closeness to the ADC reading endpoints.
                if (voltage < k_cell_open_threshold || voltage > (k_adc_vref - k_cell_open_threshold)) {
                    // Cell tap is bad.
                    continue;
                }

                // Cell tap is connected. Apply parasitic capacitance correction factor.
                cell_tap_bitet |= 1u << index;
                voltages[index] = voltage - s_correction_table[index];

                // Check if the cell tap is degraded (noisy).
                if (adc_range > k_cell_degraded_threshold) {
                    degraded_bitset |= 1u << index;
                }
            }
        }

        // Copy the data in a critical section to make sure the copy is atomic and the data is self-consistent.
        freertos::in_critical_section([&] {
            s_cell_tap_bitset = cell_tap_bitet;
            s_degraded_bitset = degraded_bitset;
            std::copy(voltages.begin(), voltages.end(), s_voltages.begin());
        });

        const auto period = s_is_charging ? 2000 : 100;
        vTaskDelayUntil(&last_schedule_time, pdMS_TO_TICKS(period));
    }
}

std::optional<std::int8_t> calculate_thermistor(std::uint16_t rail_voltage, std::uint16_t voltage, float beta) {
    // Check if the voltage measurement is viable.
    if (voltage < k_thermistor_range_threshold || voltage > (rail_voltage - k_thermistor_range_threshold)) {
        return std::nullopt;
    }

    // Thermistor is connected, so we can calculate the temperature.
    // TODO: Use a lookup table/don't use floats here.
    const auto resistance = (rail_voltage * 1000) / voltage - 1000;
    const float t0 = 1.0f / 298.15f;
    const float temperature = 1.0f / (t0 + beta * std::log(static_cast<float>(resistance) / 1000.0f)) - 273.15f;
    return static_cast<std::int8_t>(temperature);
}

void sample_temperatures_task(void *) {
    // Configure analog inputs.
    s_rail_sample.configure(hal::GpioInputMode::Analog);
    s_ref_sample.configure(hal::GpioInputMode::Analog);
    for (const auto &pin : s_mux_sample) {
        pin.configure(hal::GpioInputMode::Analog);
    }

    // Configure open-drain enable output. This pin controls the output enable line for each MUX, as well the P-channel
    // MOSFET powering the thermistors.
    s_mux_en.configure(hal::GpioOutputMode::OpenDrain, hal::GpioOutputSpeed::Max2);
    hal::gpio_set(s_mux_en);

    // Configure commoned MUX selection pins.
    for (const auto &pin : s_mux_control) {
        pin.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    }

    hal::adc_init(ADC1, 5);
    hal::adc_sequence_channel(ADC1, 1, 1, 0b010u);
    hal::adc_sequence_channel(ADC1, 2, 2, 0b010u);
    hal::adc_sequence_channel(ADC1, 3, 3, 0b010u);
    hal::adc_sequence_channel(ADC1, 4, 4, 0b010u);
    hal::adc_sequence_channel(ADC1, 5, 7, 0b010u);

    std::array<std::uint16_t, 5> adc_buffer{};
    hal::adc_init_dma(adc_buffer);

    DMA1_Channel1->CCR |= DMA_CCR_TCIE;
    hal::enable_irq(DMA1_Channel1_IRQn, 7);

    TickType_t last_schedule_time = xTaskGetTickCount();
    while (true) {
        const auto period = s_is_charging ? 2000 : 1000;
        vTaskDelayUntil(&last_schedule_time, pdMS_TO_TICKS(period));

        std::uint32_t thermistor_bitset = 0;
        std::array<std::int8_t, k_thermistor_count> temperatures{};
        for (std::uint32_t selection = 0; selection < 8; selection++) {
            // Set A, B, and C MUX selection outputs.
            s_mux_control[0].write((selection & 0b1u) != 0);
            s_mux_control[1].write((selection & 0b10u) != 0);
            s_mux_control[2].write((selection & 0b100u) != 0);

            // Enable thermistors.
            for (const auto &pin : s_mux_sample) {
                pin.configure(hal::GpioInputMode::Analog);
            }
            hal::gpio_reset(s_mux_en);

            // Settle delay.
            // TODO: Lower this.
            vTaskDelay(pdMS_TO_TICKS(50));

            hal::adc_start(ADC1);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            hal::gpio_set(s_mux_en);
            for (const auto &pin : s_mux_sample) {
                pin.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
            }

            const auto rail_voltage = (k_mcu_vref * adc_buffer[0]) >> 12;
            for (std::uint32_t mux = 0; mux < 3; mux++) {
                const auto index = k_thermistor_mapping[selection][mux];
                const auto beta = 1.0f / (index < 4 ? 3350.0f : 3950.0f);
                const auto voltage = (k_mcu_vref * adc_buffer[mux + 1]) >> 12;
                const auto temperature = calculate_thermistor(rail_voltage, voltage, beta);
                if (temperature) {
                    // The temperature reading is viable.
                    s_thermistor_bitset |= 1u << index;
                    temperatures[index] = *temperature;
                }
            }
        }

        // Copy the data in a critical section to make sure the copy is atomic and the data is self-consistent.
        freertos::in_critical_section([&] {
            // Re-calculate the last rail voltage for logging.
            s_rail_voltage = (k_mcu_vref * adc_buffer[0]) >> 12;

            // Calculate the external reference voltage. The input has a 2x divider.
            s_ref_voltage = (k_mcu_vref * adc_buffer[4]) >> 11;

            // Copy sampled thermistor data.
            s_thermistor_bitset = thermistor_bitset;
            std::copy(temperatures.begin(), temperatures.end(), s_temperatures.begin());
        });
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
    std::uint16_t balance_bitset = 0;
    while (bytes.size() > stream.head() + sizeof(std::uint32_t)) {
        const auto address = stream.read_byte();
        const auto bitset = stream.read_be<std::uint16_t>();
        if (!address || !bitset) {
            ++s_i2c_error_count;
            return;
        }
        if (*address == s_i2c_address) {
            balance_bitset = *bitset;
        }
    }
    const auto expected_crc = stream.read_be<std::uint32_t>();
    if (!expected_crc) {
        ++s_i2c_error_count;
        return;
    }

    // Compute and compare the actual CRC.
    const auto crc = freertos::in_critical_section([&] {
        return hal::crc_compute(bytes.subspan(0, bytes.size() - 4));
    });
    if (expected_crc != crc) {
        ++s_i2c_error_count;
        return;
    }

    s_is_charging.store(*mode_byte == 0x55);

    std::uint16_t reversed_bitset = 0;
    for (std::size_t i = 0; i < 16; i++) {
        if ((balance_bitset & (1u << i)) != 0) {
            reversed_bitset |= (1u << (15 - i));
        }
    }
    s_balance_bitset.store(reversed_bitset);
}

void i2c_listen() {
    hal::enable_irq(I2C1_EV_IRQn, 6);
    hal::enable_irq(I2C1_ER_IRQn, 6);
    if (s_i2c_sm.state() == i2c::State::Error) {
        ++s_i2c_error_count;
        s_i2c_sm.init();
    }
    // Listen on our configured address and the general call address.
    s_i2c_sm.listen(s_i2c_address, true);
}

void cmd_task(void *) {
    s_i2c_address = 0x40u | ~(GPIOA->IDR >> 8u) & 0xfu;

    s_i2c_sm.init();
    while (true) {
        std::array<std::uint8_t, 64> cmd_bytes{};
        const auto cmd_length =
            xMessageBufferReceive(*s_cmd_queue, cmd_bytes.data(), cmd_bytes.size(), pdMS_TO_TICKS(k_sleep_timeout));
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

        // Reconfigure SCL as a regular input for use as an external event. Also reconfigure SDA to avoid the STM
        // driving it low and upsetting the isolator.
        s_scl.configure(hal::GpioInputMode::Floating);
        s_sda.configure(hal::GpioInputMode::Floating);

        // Setup external event on SCL (PB6).
        // TODO: Make a HAL function for this.
        AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI6_PB;
        EXTI->EMR |= EXTI_EMR_MR6;
        EXTI->FTSR |= EXTI_FTSR_TR6;

        // Enter stop mode. Disable SysTick to avoid an STM errata.
        SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
        // hal::enter_stop_mode(hal::WakeupSource::Event);
        SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

        // TODO
        vTaskDelay(pdMS_TO_TICKS(100));

        // Woken up from stop mode - enable the frontend and reference.
        hal::gpio_set(s_afe_en, s_ref_en, s_led);

        // Clear saved command.
        s_is_charging.store(false);
        s_balance_bitset.store(0);

        // Listen on I2C.
        s_scl.configure(hal::GpioOutputMode::AlternateOpenDrain, hal::GpioOutputSpeed::Max2);
        s_sda.configure(hal::GpioOutputMode::AlternateOpenDrain, hal::GpioOutputSpeed::Max2);
        i2c_listen();

        // Wake the external ADC.
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
        std::array<std::optional<std::pair<std::uint16_t, std::uint16_t>>, k_cell_count> samples;
        std::fill(s_correction_table.begin(), s_correction_table.end(), 0);
        sample_voltages_raw(samples, true);
        for (std::size_t i = 0; i < k_cell_count; i++) {
            if (const auto sample = samples[i]) {
                s_correction_table[i] = (sample->first + 127) / 128;
            }
        }

        // TODO
        vTaskDelay(pdMS_TO_TICKS(1000000));
    }
}

void swd_task(void *) {
    TickType_t last_schedule_time = xTaskGetTickCount();
    while (true) {
        hal::swd_printf("--------------------------------\n");
        hal::swd_printf("Mode: %s\n", s_is_charging ? "charging" : "discharging");
        hal::swd_printf("Balance: 0x%x\n", s_balance_bitset.load());
        hal::swd_printf("Balance temperatures: [%d, %d, %d, %d]\n", s_temperatures[0], s_temperatures[1],
                        s_temperatures[2], s_temperatures[3]);
        hal::swd_printf("RAIL voltage: %u\n", s_rail_voltage);
        hal::swd_printf("REF voltage: %u\n", s_ref_voltage);
        hal::swd_printf("I2C error count: %u\n", s_i2c_error_count.load());
        hal::swd_printf("C[1, 8]:  [%u, %u, %u, %u, %u, %u, %u, %u]\n", s_correction_table[0], s_correction_table[1],
                        s_correction_table[2], s_correction_table[3], s_correction_table[4], s_correction_table[5],
                        s_correction_table[6], s_correction_table[7]);
        hal::swd_printf("C[9, 16]: [%u, %u, %u, %u, %u, %u, %u, %u]\n", s_correction_table[8], s_correction_table[9],
                        s_correction_table[10], s_correction_table[11], s_correction_table[12], s_correction_table[13],
                        s_correction_table[14], s_correction_table[15]);
        hal::swd_printf("V[1, 8]:  [%u, %u, %u, %u, %u, %u, %u, %u]\n", s_voltages[0], s_voltages[1], s_voltages[2],
                        s_voltages[3], s_voltages[4], s_voltages[5], s_voltages[6], s_voltages[7]);
        hal::swd_printf("V[9, 16]: [%u, %u, %u, %u, %u, %u, %u, %u]\n", s_voltages[8], s_voltages[9], s_voltages[10],
                        s_voltages[11], s_voltages[12], s_voltages[13], s_voltages[14], s_voltages[15]);
        hal::swd_printf("T[1, 10]: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d]\n", s_temperatures[4], s_temperatures[5],
                        s_temperatures[6], s_temperatures[7], s_temperatures[8], s_temperatures[9], s_temperatures[10],
                        s_temperatures[11], s_temperatures[12], s_temperatures[13]);
        hal::swd_printf("T[11, 20]: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d]\n", s_temperatures[14], s_temperatures[15],
                        s_temperatures[16], s_temperatures[17], s_temperatures[18], s_temperatures[19],
                        s_temperatures[20], s_temperatures[21], s_temperatures[22], s_temperatures[23]);
        vTaskDelayUntil(&last_schedule_time, pdMS_TO_TICKS(1000));
    }
}

} // namespace

extern "C" void DMA1_Channel1_IRQHandler() {
    BaseType_t higher_priority_task_woken = pdFALSE;
    DMA1->IFCR |= DMA_IFCR_CTCIF1;
    xTaskNotifyFromISR(*s_sample_temperatures_task, 1, eIncrement, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

extern "C" void I2C1_EV_IRQHandler() {
    if (!s_i2c_sm.event()) {
        // State not changed.
        return;
    }

    BaseType_t higher_priority_task_woken = pdFALSE;
    const auto state = s_i2c_sm.state();
    if (state == i2c::State::SlaveRx) {
        s_i2c_sm.set_buffer(std::span(s_i2c_buffer).subspan(0, 16));
    } else if (state == i2c::State::SlaveRxFinish) {
        if (xMessageBufferSendFromISR(*s_cmd_queue, s_i2c_buffer.data(), s_i2c_sm.head(),
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
        s_i2c_sm.set_buffer(stream.bytes());
    }

    if (state == i2c::State::Idle || state == i2c::State::SlaveRxFinish || state == i2c::State::Error ||
        state == i2c::State::NoAck) {
        i2c_listen();
    }
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

extern "C" void I2C1_ER_IRQHandler() {
    s_i2c_sm.error();
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

    s_cmd_queue.init();

    s_cmd_task.init(&cmd_task, "cmd", 4);
    s_sample_voltages_task.init(&sample_voltages_task, "voltages", 3);
    s_sample_temperatures_task.init(&sample_temperatures_task, "temperatures", 2);
    if constexpr (k_enable_debug_logs) {
        s_swd_task.init(&swd_task, "swd", 1);
    }
    vTaskStartScheduler();
}
