#include <bms/error.hh>
#include <hal.hh>
#include <stm32f103xb.h>
#include <util.hh>

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include <algorithm>
#include <bit>
#include <cstdint>
#include <optional>

// TODO: Send CAN status messages.
// TODO: Implement current sensing.
// TODO: Implement EEPROM config.
// TODO: Receive CAN ready to drive and config update messages.
// TODO: Implement inverter shutdown.
// TODO: Control LED via DMA.
// TODO: Charger control.
// TODO: SOC estimation.

namespace {

/**
 * @brief Whether to enable the SWD debug logging task.
 */
constexpr bool k_enable_debug_logs = false;

/**
 * @brief The I2C address of the first segment.
 */
constexpr std::uint8_t k_segment_address_start = 0x40;

/**
 * @brief The maximum number of connected segments.
 */
constexpr std::size_t k_max_segment_count = 10;

/**
 * @brief The maximum number of cells per segment.
 */
constexpr std::size_t k_max_cell_count = 12;

/**
 * @brief The maximum number of temperature sensors per segment.
 */
constexpr std::size_t k_max_temperature_count = 23;

/**
 * @brief The minimum allowed value for the undervoltage threshold in 100 uV resolution.
 */
constexpr std::uint16_t k_undervoltage_threshold_limit = 25000;

/**
 * @brief The maximum allowed value for the overvoltage threshold in 100 uV resolution.
 */
constexpr std::uint16_t k_overvoltage_threshold_limit = 42500;

/**
 * @brief The minimum allowed value for the undertemperature threshold in whole degrees.
 */
constexpr std::int8_t k_undertemperature_threshold_limit = -20;

/**
 * @brief The maximum allowed value for the overtemperature threshold in whole degrees.
 */
constexpr std::int8_t k_overtemperature_threshold_limit = 100;

/**
 * @brief Hard-coded value of the on-board precision voltage reference in 100 uV resolution.
 */
constexpr std::uint16_t k_reference_voltage = 40960;

/**
 * @brief Configuration header magic number.
 */
constexpr std::uint32_t k_config_magic = 0x6c72d132;

/**
 * @brief Supervisor task scheduling period in milliseconds.
 */
constexpr std::uint32_t k_supervisor_period = 10;

/**
 * @brief Segment sampling task scheduling period in milliseconds.
 */
constexpr std::uint32_t k_segment_sample_period = 50;

/**
 * @brief MCU temperature and 3V3 rail sampling task scheduling period in milliseconds.
 */
constexpr std::uint32_t k_mcu_sample_period = 200;

/**
 * @brief Whether shutdowns can be cancelled if the fault clears within the delay grace period.
 */
constexpr bool k_allow_shutdown_cancellation = true;

/**
 * @brief The amount of time to wait in milliseconds before opening the shutdown circuit in the event of a controlled
 * shutdown event.
 *
 * This includes most battery parameter shutdowns (overvoltage, overtemperature, etc.), but not watchdog triggers, MCU
 * resets, or extreme overcurrent events.
 */
constexpr std::uint32_t k_shutdown_delay = 200;

/**
 * @brief The maximum time the BMS has to react to a fault in milliseconds before it must react before opening the
 * shutdown circuit.
 */
constexpr std::uint32_t k_maximum_reaction_time = 450;

// The supervisor task must have enough time to run after a shutdown has been started.
static_assert(k_maximum_reaction_time > k_shutdown_delay + k_supervisor_period * 2);

/**
 * @brief The calculated upper limit in milliseconds of task deadline overrun.
 */
constexpr std::uint32_t k_schedule_tolerance = pdMS_TO_TICKS(k_maximum_reaction_time - k_shutdown_delay);

struct Config {
    std::uint16_t undervoltage_threshold;
    std::uint16_t overvoltage_threshold;
    std::int8_t undertemperature_threshold;
    std::int8_t overtemperature_threshold;
    std::uint8_t expected_cell_count;
    std::uint8_t minimum_thermistor_count;
};

class Segment {
    std::array<std::optional<std::uint16_t>, k_max_cell_count> m_cell_voltages;
    std::array<std::optional<std::int8_t>, k_max_temperature_count> m_temperatures;
    std::uint16_t m_degraded_bitset{};
    std::uint16_t m_rail_voltage{};
    bms::SegmentErrorFlags m_error_flags{};
    TickType_t m_last_update_time{};

public:
    void update(util::Stream &stream);

    const auto &cell_voltages() const { return m_cell_voltages; }
    const auto &temperatures() const { return m_temperatures; }
    std::uint16_t degraded_bitset() const { return m_degraded_bitset; }
    std::uint16_t rail_voltage() const { return m_rail_voltage; }
    bms::SegmentErrorFlags error_flags() const { return m_error_flags; }
    TickType_t last_update_time() const { return m_last_update_time; }
};

// Global config.
Config s_config;

// Global array of segments with associated mutex.
std::array<Segment, k_max_segment_count> s_segments;
SemaphoreHandle_t s_segments_mutex;

// Sampled values.
std::uint16_t s_vdd_voltage = 0;
std::uint16_t s_lvs_voltage = 0;
std::int8_t s_mcu_temperature = 0;
SemaphoreHandle_t s_mcu_mutex;

// Task timing.
TickType_t s_last_segment_sample_time = 0;
TickType_t s_last_mcu_sample_time = 0;

hal::Gpio s_lvs_reading(hal::GpioPort::A, 1);

hal::Gpio s_shutdown(hal::GpioPort::B, 1);
hal::Gpio s_led(hal::GpioPort::B, 5);

// I2C pins.
hal::Gpio s_scl_1(hal::GpioPort::B, 6);
hal::Gpio s_sda_1(hal::GpioPort::B, 7);
hal::Gpio s_scl_2(hal::GpioPort::B, 10);
hal::Gpio s_sda_2(hal::GpioPort::B, 11);

/**
 * @brief The supervisor task for the BMS which is responsible for the shutdown output and watchdog feeding.
 *
 * This task has the highest priority and oversees deadlines for all of the other tasks. If any other task stops being
 * scheduled (and therefore stops monitoring the battery properly), this task will notice and open the shutdown output.
 * If this task stops being scheduled, or the whole MCU locks up, this task will stop feeding the watchdog which will
 * cause it to timeout, open the shutdown output, and reset the MCU.
 */
void supervisor_task(void *) {
    // Come out of shutdown.
    // TODO: Do this after a grace period.
    hal::gpio_set(s_shutdown);

    TickType_t last_schedule_time = xTaskGetTickCount();
    std::optional<std::size_t> expected_segment_count;
    std::optional<TickType_t> shutdown_request_time;
    while (true) {
        // We calculate the latest master flags based on the most recent master state and segment data.
        bms::MasterErrorFlags master_flags;

        // Check for missed deadlines for all of the other tasks.
        const auto current_time = xTaskGetTickCount();
        if (current_time - (s_last_segment_sample_time - k_segment_sample_period) >= k_schedule_tolerance) {
            master_flags.set(bms::MasterError::DeadlineOverrun);
        }
        if (current_time - (s_last_mcu_sample_time - k_mcu_sample_period) >= k_schedule_tolerance) {
            master_flags.set(bms::MasterError::DeadlineOverrun);
        }

        // Check segment data.
        std::size_t ready_segment_count = 0;
        xSemaphoreTake(s_segments_mutex, portMAX_DELAY);
        for (const auto &segment : s_segments) {
            if (!segment.error_flags().is_set(bms::SegmentError::Disconnected)) {
                ready_segment_count++;

                // Check for stale data in case of unreliable communication, for example.
                if (current_time - segment.last_update_time() >= k_schedule_tolerance) {
                    master_flags.set(bms::MasterError::DeadlineOverrun);
                }
                if (segment.error_flags().any_set()) {
                    master_flags.set(bms::MasterError::SegmentError);
                }
            }
        }
        xSemaphoreGive(s_segments_mutex);

        // Check we have the right amount of segments connected.
        if (expected_segment_count && *expected_segment_count != ready_segment_count) {
            master_flags.set(bms::MasterError::BadSegmentCount);
        }

        // TODO: Check MCU values.

        // Tentatively start a shutdown if any error flags are set.
        const bool should_shutdown = master_flags.any_set();
        if (should_shutdown && !shutdown_request_time) {
            // New shutdown request.
            shutdown_request_time.emplace(xTaskGetTickCount());
        } else if (!should_shutdown && k_allow_shutdown_cancellation) {
            // Cancel the request.
            shutdown_request_time.reset();
        }

        // Actually open the shutdown circuit when the grace period is over. The shutdown state can currently not be
        // left.
        if (shutdown_request_time && xTaskGetTickCount() - *shutdown_request_time >= pdMS_TO_TICKS(k_shutdown_delay)) {
            hal::gpio_reset(s_shutdown);
        }

        xTaskDelayUntil(&last_schedule_time, pdMS_TO_TICKS(k_supervisor_period));
    }
}

void Segment::update(util::Stream &stream) {
    m_error_flags.clear();
    if (stream.empty()) {
        m_error_flags.set(bms::SegmentError::Disconnected);
        return;
    }

    const auto thermistor_bitset = stream.read_be<std::uint32_t>();
    if (!thermistor_bitset) {
        m_error_flags.set(bms::SegmentError::UnreliableCommunication);
        return;
    }

    const auto cell_tap_bitset = stream.read_be<std::uint16_t>();
    if (!cell_tap_bitset) {
        m_error_flags.set(bms::SegmentError::UnreliableCommunication);
        return;
    }

    const auto degraded_bitset = stream.read_be<std::uint16_t>();
    if (!degraded_bitset) {
        m_error_flags.set(bms::SegmentError::UnreliableCommunication);
        return;
    }

    const auto rail_voltage = stream.read_be<std::uint16_t>();
    if (!rail_voltage) {
        m_error_flags.set(bms::SegmentError::UnreliableCommunication);
        return;
    }

    std::array<std::uint16_t, k_max_cell_count> cell_voltages{};
    for (std::size_t i = 0; i < k_max_cell_count; i++) {
        const auto voltage = stream.read_be<std::uint16_t>();
        if (!voltage) {
            m_error_flags.set(bms::SegmentError::UnreliableCommunication);
            return;
        }
        cell_voltages[i] = *voltage;
    }

    std::array<std::int8_t, k_max_temperature_count> temperatures{};
    for (std::size_t i = 0; i < k_max_temperature_count; i++) {
        const auto temperature = stream.read_be<std::int8_t>();
        if (!temperature) {
            m_error_flags.set(bms::SegmentError::UnreliableCommunication);
            return;
        }
        temperatures[i] = *temperature;
    }

    const auto segment_ready = stream.read_byte();
    if (!segment_ready) {
        m_error_flags.set(bms::SegmentError::UnreliableCommunication);
        return;
    }
    if (*segment_ready != 1) {
        m_error_flags.set(bms::SegmentError::Disconnected);
        return;
    }

    // Update all data in one go once we know it's valid in terms of checksum and length validity.
    for (std::size_t i = 0; i < k_max_cell_count; i++) {
        if ((*cell_tap_bitset & (1u << i)) != 0) {
            m_cell_voltages[i] = cell_voltages[i];
        } else {
            m_cell_voltages[i].reset();
        }
    }
    for (std::size_t i = 0; i < k_max_temperature_count; i++) {
        if ((*thermistor_bitset & (1u << i)) != 0) {
            m_temperatures[i] = temperatures[i];
        } else {
            m_temperatures[i].reset();
        }
    }
    m_degraded_bitset = *degraded_bitset;
    m_rail_voltage = *rail_voltage;
    m_last_update_time = xTaskGetTickCount();

    // Cell count should be exactly right.
    if (std::popcount(*cell_tap_bitset) != s_config.expected_cell_count) {
        m_error_flags.set(bms::SegmentError::BadCellCount);
    }

    // Thermistors may be allowed to drop below the maximum.
    if (std::popcount(*thermistor_bitset) < s_config.minimum_thermistor_count) {
        m_error_flags.set(bms::SegmentError::BadThermistorCount);
    }

    // Don't accept all degraded cell voltage readings.
    if (std::popcount(m_degraded_bitset) >= s_config.expected_cell_count) {
        m_error_flags.set(bms::SegmentError::UnreliableMeasurement);
    }

    // Rail voltage needs to be right otherwise measurements are meaningless.
    if (m_rail_voltage < 31000 || m_rail_voltage > 35000) {
        m_error_flags.set(bms::SegmentError::BadRailVoltage);
        m_error_flags.set(bms::SegmentError::UnreliableMeasurement);
    }

    // Clamp config limits just in-case.
    const auto undervoltage_threshold = std::max(s_config.undervoltage_threshold, k_undervoltage_threshold_limit);
    const auto overvoltage_threshold = std::min(s_config.overvoltage_threshold, k_overvoltage_threshold_limit);
    for (const auto voltage : m_cell_voltages) {
        if (voltage) {
            if (*voltage < undervoltage_threshold) {
                m_error_flags.set(bms::SegmentError::Undervoltage);
            }
            if (*voltage > overvoltage_threshold) {
                m_error_flags.set(bms::SegmentError::Overvoltage);
            }
        }
    }

    const auto undertemperature_threshold =
        std::max(s_config.undertemperature_threshold, k_undertemperature_threshold_limit);
    const auto overtemperature_threshold =
        std::min(s_config.overtemperature_threshold, k_overtemperature_threshold_limit);
    for (const auto temperature : m_temperatures) {
        if (temperature) {
            if (*temperature < undertemperature_threshold) {
                m_error_flags.set(bms::SegmentError::Undertemperature);
            }
            if (*temperature > overtemperature_threshold) {
                m_error_flags.set(bms::SegmentError::Overtemperature);
            }
        }
    }
}

bool read_i2c(I2C_TypeDef *i2c, std::uint8_t address, std::span<std::uint8_t> data) {
    // Bus should already be idle, only allow 1 ms.
    if (hal::i2c_wait_idle(i2c, 1) != hal::I2cStatus::Ok) {
        return false;
    }
    // TODO: Should use interrupts.
    const bool ok = hal::i2c_master_read(i2c, address, data, 100) == hal::I2cStatus::Ok;
    hal::i2c_stop(i2c);
    return ok;
}

void sample_segment(std::size_t index) {
    // TODO: Don't ready a fixed amount of bytes and add a checksum.
    std::array<std::uint8_t, 60> bytes{};
    const auto address = k_segment_address_start + index;
    const bool present = read_i2c(I2C1, address, bytes) || read_i2c(I2C2, address, bytes);

    xSemaphoreTake(s_segments_mutex, portMAX_DELAY);
    if (present) {
        util::Stream stream(bytes);
        s_segments[index].update(stream);
    } else {
        // No I2C response.
        util::Stream stream({});
        s_segments[index].update(stream);
    }
    xSemaphoreGive(s_segments_mutex);
}

void sample_segments_task(void *) {
    s_last_segment_sample_time = xTaskGetTickCount();
    while (true) {
        // Wakeup all segments by effectively pulling SCL low.
        I2C1->CR1 &= ~I2C_CR1_PE;
        I2C2->CR1 &= ~I2C_CR1_PE;
        for (const auto &pin : {s_scl_1, s_scl_2}) {
            pin.configure(hal::GpioOutputMode::OpenDrain, hal::GpioOutputSpeed::Max2);
            pin.configure(hal::GpioOutputMode::AlternateOpenDrain, hal::GpioOutputSpeed::Max2);
        }

        // Allow some time for segment wake-up.
        vTaskDelay(pdMS_TO_TICKS(1));

        // Reinitialise the I2C peripherals.
        hal::i2c_init(I2C1, std::nullopt);
        hal::i2c_init(I2C2, std::nullopt);

        for (std::size_t index = 0; index < k_max_segment_count; index++) {
            sample_segment(index);
        }

        xTaskDelayUntil(&s_last_segment_sample_time, pdMS_TO_TICKS(k_segment_sample_period));
    }
}

void sample_mcu_task(void *) {
    // Sequence the LVS voltage reading as well as the STM's internal temperature sensor and voltage reference.
    // TODO: Sample external reference voltage on hardware revision B.
    hal::adc_init(ADC1, 3);
    hal::adc_sequence_channel(ADC1, 1, 1, 0b010u);
    hal::adc_sequence_channel(ADC1, 2, 16, 0b111u);
    hal::adc_sequence_channel(ADC1, 3, 17, 0b111u);

    std::array<std::uint16_t, 3> adc_buffer{};
    hal::adc_init_dma(adc_buffer);

    s_last_mcu_sample_time = xTaskGetTickCount();
    while (true) {
        // Calculate VDD rail voltage from the STM's internal 1.2 volt band gap reference.
        const auto vdd = (1200 * 4096) / adc_buffer[2];

        // Calculate LVS input voltage. The input has a 5.3x divider.
        const auto lvs_voltage = (((vdd * adc_buffer[0]) >> 12) * 53) / 10;

        // Calculate an approximate temperature using constants from the datasheet.
        const auto temperature_voltage = (vdd * adc_buffer[1]) >> 12;
        const auto temperature = ((1430 - temperature_voltage) * 10) / 43 + 25;

        // Update global values.
        xSemaphoreTake(s_mcu_mutex, portMAX_DELAY);
        s_vdd_voltage = vdd;
        s_lvs_voltage = lvs_voltage;
        s_mcu_temperature = temperature;
        xSemaphoreGive(s_mcu_mutex);

        hal::adc_start(ADC1);
        xTaskDelayUntil(&s_last_mcu_sample_time, pdMS_TO_TICKS(k_mcu_sample_period));
    }
}

void swd_task(void *) {
    TickType_t last_schedule_time = xTaskGetTickCount();
    while (true) {
        hal::swd_printf("--------------------------------\n");
        hal::swd_printf("Shutdown activated: %s\n", !s_shutdown.read() ? "yes" : "no");

        xSemaphoreTake(s_mcu_mutex, portMAX_DELAY);
        hal::swd_printf("LVS voltage: %u\n", s_lvs_voltage);
        hal::swd_printf("MCU voltage: %u\n", s_vdd_voltage);
        hal::swd_printf("MCU temperature: %d\n", s_mcu_temperature);
        xSemaphoreGive(s_mcu_mutex);

        const auto current_time = xTaskGetTickCount();
        xSemaphoreTake(s_segments_mutex, portMAX_DELAY);
        for (std::size_t i = 0; i < s_segments.size(); i++) {
            const auto &segment = s_segments[i];
            if (segment.error_flags().is_set(bms::SegmentError::Disconnected)) {
                continue;
            }

            hal::swd_printf("Segment %u (sampled %u ms ago)\n", i, current_time - segment.last_update_time());
            hal::swd_printf("  Error flags: 0x%x\n", static_cast<std::uint32_t>(segment.error_flags()));
            hal::swd_printf("  Rail voltage: %u\n", segment.rail_voltage());
            hal::swd_printf("  Degraded bitset: 0x%x\n", segment.degraded_bitset());
            hal::swd_printf("  Voltages: [");
            for (bool first = true; const auto &voltage : segment.cell_voltages()) {
                if (!first) {
                    hal::swd_printf(", ");
                }
                first = false;
                if (voltage) {
                    hal::swd_printf("%u", *voltage);
                } else {
                    hal::swd_printf("X");
                }
            }
            hal::swd_printf("]\n  Temperatures: [");
            for (bool first = true; const auto &temperature : segment.temperatures()) {
                if (!first) {
                    hal::swd_printf(", ");
                }
                first = false;
                if (temperature) {
                    hal::swd_printf("%d", *temperature);
                } else {
                    hal::swd_printf("X");
                }
            }
            hal::swd_printf("]\n");
        }
        xSemaphoreGive(s_segments_mutex);

        xTaskDelayUntil(&last_schedule_time, pdMS_TO_TICKS(1000));
    }
}

} // namespace

void vApplicationIdleHook() {
    // TODO: Could disable some peripherals like I2C.
    // TODO: Test power consumption of this and verify that it doesn't affect timing.
    hal::enter_sleep_mode(hal::WakeupSource::Interrupt);
}

void app_main() {
    s_lvs_reading.configure(hal::GpioInputMode::Analog);
    s_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

    // Configure I2C pins for peripheral use.
    for (const auto &pin : {s_scl_1, s_sda_1, s_scl_2, s_sda_2}) {
        pin.configure(hal::GpioOutputMode::AlternateOpenDrain, hal::GpioOutputSpeed::Max2);
    }

    // Shutdown output is push-pull but also has an external pull-down in case of MCU failure.
    s_shutdown.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

    // TODO: Read config from EEPROM.
    s_config = {
        35000, 42000, -20, 24, 12, 3,
    };

    // Initialise both I2C buses and SPI for the ADC.
    hal::i2c_init(I2C1, std::nullopt);
    hal::i2c_init(I2C2, std::nullopt);
    hal::spi_init_master(SPI2, SPI_CR1_BR_2);

    // Todo: Configure OC_P and OC_N pins as interrupts for immediate shutdown.

    // Lock pins whose configurations don't need to change.
    hal::gpio_lock(s_lvs_reading, s_shutdown, s_led);

    // Initialise segment data.
    for (auto &segment : s_segments) {
        util::Stream stream({});
        segment.update(stream);
    }

    s_segments_mutex = xSemaphoreCreateMutex();
    s_mcu_mutex = xSemaphoreCreateMutex();

    // Create all tasks.
    // TODO: Think about stack sizes and priorities more.
    xTaskCreate(&supervisor_task, "supervisor", 128, nullptr, 4, nullptr);
    xTaskCreate(&sample_segments_task, "sample_segs", 256, nullptr, 3, nullptr);
    xTaskCreate(&sample_mcu_task, "sample_mcu", 128, nullptr, 2, nullptr);
    if constexpr (k_enable_debug_logs) {
        xTaskCreate(&swd_task, "swd", 128, nullptr, 1, nullptr);
    }

    // Start the scheduler which we shouldn't return from.
    vTaskStartScheduler();
    while (true) {
        // If we somehow get here, signal shutdown and let the watchdog timeout.
        hal::gpio_reset(s_shutdown);
    }
}
