#include <bms/can_messages.hh>
#include <bms/error.hh>
#include <can.hh>
#include <config.hh>
#include <freertos.hh>
#include <hal.hh>
#include <i2c.hh>
#include <stm32f103xb.h>
#include <util.hh>

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <optional>

// TODO: Send CAN status messages.
// TODO: Implement current sensing for negative sensor and plausibility checks.
// TODO: Receive CAN ready to drive message.
// TODO: Implement inverter shutdown.
// TODO: Control LED via DMA.
// TODO: Charger control.
// TODO: SOC estimation.
// TODO: Enable stack overflow detection.
// TODO: EEPROM page for self test (PCBA).

using namespace bms;

namespace {

/**
 * @brief Whether to enable the SWD debug logging task.
 */
constexpr bool k_enable_debug_logs = false;

/**
 * @brief The I2C address of the EEPROM.
 */
constexpr std::uint8_t k_eeprom_address = 0x50;

/**
 * @brief The size of an EEPROM page in bytes.
 */
constexpr std::uint16_t k_eeprom_page_size = 32;

/**
 * @brief I2C message size received from a segment in bytes.
 */
constexpr std::size_t k_segment_response_size = 63;

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
constexpr std::uint16_t k_adc_vref = 40960;

/**
 * @brief Hard-coded value of the 3V3 rail powering the STM's ADC in 1 mV resolution.
 */
constexpr std::uint32_t k_mcu_vref = 3300;

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
 * @brief MCU ADC sampling task scheduling period in milliseconds.
 */
constexpr std::uint32_t k_mcu_sample_period = 200;

/**
 * @brief Current sensing voltage low pass filter time period in seconds.
 */
constexpr float k_current_sense_tau = 0.5f;

/**
 * @brief Current sensing zero voltage tracking low pass filter time period in seconds.
 */
constexpr float k_current_sense_zero_tau = 3.5f;

/**
 * @brief Magnitude of current which can be rounded down to zero current in amps.
 */
constexpr float k_current_sense_zero_threshold = 0.1f;

/**
 * @brief Current sensor sensitivity in volts per amp.
 */
constexpr float k_current_sense_sensitivity = 3.2f / 1000.0f;

/**
 * @brief Current sensor ideal voltage output at 0 amps.
 */
constexpr float k_current_sense_ideal_zero = 2.5f;

/**
 * @brief Whether shutdown assertion requests can be cancelled if the fault clears within the delay grace period.
 */
constexpr bool k_allow_shutdown_cancellation = true;

/**
 * @brief The amount of time to wait in milliseconds before asserting shutdown in the event of a controlled shutdown
 * event.
 *
 * This includes most battery parameter shutdowns (overvoltage, overtemperature, etc.), but not watchdog triggers, MCU
 * resets, or extreme overcurrent events. The benefit of this parameter is that it allows the external load, such as an
 * inverter, to gracefully reduce power before the contactors open.
 */
constexpr std::uint32_t k_shutdown_assert_delay = 200;

/**
 * @brief The amount of time to wait in milliseconds before deasserting shutdown after all error flags are no longer
 * present.
 */
constexpr std::uint32_t k_shutdown_deassert_delay = 1000;

/**
 * @brief The amount of time to wait in milliseconds before deasserting shutdown after startup, provided that
 * there are no error flags.
 *
 * The BMS starts with shutdown asserted in order to allow for a delay period in the event of a watchdog trigger, power
 * failure, or other MCU resets. This parameter defines the time to wait before deasserting.
 */
constexpr std::uint32_t k_shutdown_start_delay = 100;

// The shutdown deassert delay must be at least double the shutdown assert delay to allow for some hysteresis.
static_assert(k_shutdown_deassert_delay >= k_shutdown_assert_delay * 2);

// The shutdown start delay cannot be higher than the regular deassert delay.
static_assert(k_shutdown_start_delay <= k_shutdown_deassert_delay);

/**
 * @brief The maximum time in milliseconds that the BMS has to react to a fault before it must assert shutdown.
 */
constexpr std::uint32_t k_maximum_reaction_time = 450;

// The supervisor task must have enough time to run after a shutdown has been started.
static_assert(k_maximum_reaction_time > k_shutdown_assert_delay + k_supervisor_period * 2);

/**
 * @brief The calculated upper limit in milliseconds of task deadline overrun.
 */
constexpr std::uint32_t k_schedule_tolerance = pdMS_TO_TICKS(k_maximum_reaction_time - k_shutdown_assert_delay);

struct Config {
    // Segment config.
    std::uint8_t segment_start_address;
    std::uint8_t segment_count;
    std::uint8_t cell_count;
    std::uint8_t minimum_thermistor_count;

    // Threshold config.
    std::uint16_t undervoltage_threshold;
    std::uint16_t overvoltage_threshold;
    std::int8_t undertemperature_threshold;
    std::int8_t overtemperature_threshold;

    // Header data.
    std::uint32_t magic;
    std::uint32_t counter;
    std::uint32_t crc;
};

constexpr std::array<std::uint16_t, 2> k_config_offsets{
    0,
    (sizeof(Config) + k_eeprom_page_size - 1) / k_eeprom_page_size,
};

class CurrentSensor {
    float m_filtered_voltage{k_current_sense_ideal_zero};
    float m_zero_voltage{0.0f};
    float m_current{0.0f};
    bool m_has_initial_zero{false};

public:
    void update(float voltage);

    float current() const { return m_current; }
};

class Segment {
    std::array<std::optional<std::uint16_t>, k_max_cell_count> m_cell_voltages;
    std::array<std::optional<std::int8_t>, k_max_temperature_count> m_temperatures;
    std::uint32_t m_master_error_count{};
    std::uint32_t m_slave_error_count{};
    std::uint16_t m_degraded_bitset{};
    SegmentErrorFlags m_error_flags{SegmentError::Disconnected};
    TickType_t m_last_update_time{};

public:
    void update(std::span<std::uint8_t> bytes);

    const auto &cell_voltages() const { return m_cell_voltages; }
    const auto &temperatures() const { return m_temperatures; }
    std::uint32_t master_error_count() const { return m_master_error_count; }
    std::uint32_t slave_error_count() const { return m_slave_error_count; }
    std::uint16_t degraded_bitset() const { return m_degraded_bitset; }
    SegmentErrorFlags error_flags() const { return m_error_flags; }
    TickType_t last_update_time() const { return m_last_update_time; }
};

// Active config.
Config s_config;

// Global array of segments with associated mutex.
std::array<Segment, k_max_segment_count> s_segments;
freertos::Mutex s_segments_mutex;

// Current sensing.
CurrentSensor m_positive_sensor;

// Sampled values.
std::uint16_t s_lvs_voltage = 0;
std::uint16_t s_ref_voltage = 0;
std::int8_t s_mcu_temperature = 0;
freertos::Mutex s_mcu_mutex;

// Time of shutdown assertion. Presence indicates whether shutdown is asserted.
std::optional<TickType_t> s_shutdown_time;

// I2C state machines.
i2c::StateMachine s_i2c1_sm(i2c::Bus::_1);
i2c::StateMachine s_i2c2_sm(i2c::Bus::_2);

// Tasks.
// TODO: Think about stack sizes more.
freertos::Task<128> s_supervisor_task;
freertos::Task<256> s_sample_segments_task;
freertos::Task<128> s_sample_mcu_task;
freertos::Task<256> s_config_task;
freertos::Task<128> s_swd_task;

// Task timing.
TickType_t s_last_segment_sample_time = 0;
TickType_t s_last_mcu_sample_time = 0;

// Watchdog pins.
hal::Gpio s_wdi(hal::GpioPort::A, 3);
hal::Gpio s_wds(hal::GpioPort::A, 4);

hal::Gpio s_lvs_sample(hal::GpioPort::A, 1);
hal::Gpio s_ref_sample(hal::GpioPort::A, 7);
hal::Gpio s_oc_n(hal::GpioPort::A, 11);
hal::Gpio s_oc_p(hal::GpioPort::A, 12);

hal::Gpio s_shutdown(hal::GpioPort::B, 1);
hal::Gpio s_eeprom_wc(hal::GpioPort::B, 2);
hal::Gpio s_led(hal::GpioPort::B, 5);

// I2C pins.
hal::Gpio s_scl_1(hal::GpioPort::B, 6);
hal::Gpio s_sda_1(hal::GpioPort::B, 7);
hal::Gpio s_scl_2(hal::GpioPort::B, 10);
hal::Gpio s_sda_2(hal::GpioPort::B, 11);

// Switch and ADC SPI pins for current sensing.
hal::Gpio s_current_switch(hal::GpioPort::A, 10);
hal::Gpio s_adc_cs(hal::GpioPort::A, 8);
hal::Gpio s_sck(hal::GpioPort::B, 13);
hal::Gpio s_miso(hal::GpioPort::B, 14);

std::uint32_t compute_crc(std::span<const std::uint8_t> data) {
    taskENTER_CRITICAL();
    const auto crc = hal::crc_compute(data);
    taskEXIT_CRITICAL();
    return crc;
}

/**
 * @brief Returns true if at least the specified duration of time has passed since the given start time.
 *
 * @param start the period start time in FreeRTOS tick units
 * @param duration the period duration in milliseconds
 */
bool has_elapsed(TickType_t start, std::uint32_t duration) {
    // TODO: This probably needs to round up the duration to work with tick rates < 1000 Hz?
    return xTaskGetTickCount() - start >= pdMS_TO_TICKS(duration);
}

/**
 * @brief The supervisor task for the BMS which is responsible for the shutdown output and watchdog feeding.
 *
 * This task has the highest priority and oversees deadlines for all of the other tasks. If any other task stops being
 * scheduled (and therefore stops monitoring the battery properly), this task will notice and open the shutdown output.
 * If this task stops being scheduled, or the whole MCU locks up, this task will stop feeding the watchdog which will
 * cause it to timeout, assert shutdown, and reset the MCU.
 */
void supervisor_task(void *) {
    std::optional<TickType_t> shutdown_request_time;
    std::optional<TickType_t> fault_cleared_time;

    // Start with shutdown asserted and set fault_cleared_time such that shutdown will be deasserted after the start
    // delay. If an error is flagged during this window, then fault_cleared_time will be reset.
    s_shutdown_time.emplace(0);
    fault_cleared_time.emplace(k_shutdown_start_delay - k_shutdown_deassert_delay);

    // Enable the external TPS3851 watchdog.
    hal::gpio_set(s_wds);

    // Initialise CAN on port B.
    can::init(can::Port::B, config::k_can_speed, 3);

    // Enable all IRQs after enabling the watchdog.
    hal::enable_irq(CAN1_SCE_IRQn, 6);
    hal::enable_irq(CAN1_TX_IRQn, 7);
    hal::enable_irq(I2C1_EV_IRQn, 8);
    hal::enable_irq(I2C1_ER_IRQn, 8);
    hal::enable_irq(I2C2_EV_IRQn, 9);
    hal::enable_irq(I2C2_ER_IRQn, 9);
    hal::enable_irq(CAN1_RX1_IRQn, 10);

    // The current sensing interrupts don't use RTOS functions, so can have a priority below
    // configMAX_SYSCALL_INTERRUPT_PRIORITY (5), which we do for the external interrupt on MISO and the RXNE handler to
    // ensure the fairly strict SPI timing. However, the timer interrupt which drives the current sensing is kept at a
    // low priority to make current sensing in general lower priority than CAN and segment I2C.
    hal::enable_irq(TIM3_IRQn, 10);
    hal::enable_irq(EXTI15_10_IRQn, 0);
    hal::enable_irq(SPI2_IRQn, 1);

    TickType_t last_schedule_time = xTaskGetTickCount();
    while (true) {
        // Calculate the latest master flags based on the most recent master state and segment data.
        MasterErrorFlags master_flags;

        // Check for missed deadlines for all of the other tasks.
        const auto current_time = xTaskGetTickCount();
        if (current_time - (s_last_segment_sample_time - k_segment_sample_period) >= k_schedule_tolerance) {
            master_flags.set(MasterError::DeadlineOverrun);
        }
        if (current_time - (s_last_mcu_sample_time - k_mcu_sample_period) >= k_schedule_tolerance) {
            master_flags.set(MasterError::DeadlineOverrun);
        }

        // Check if CAN is functional.
        if (!can::is_online()) {
            master_flags.set(bms::MasterError::BadCan);
        }

        // Check segment data.
        std::size_t ready_segment_count = 0;
        xSemaphoreTake(*s_segments_mutex, portMAX_DELAY);
        for (const auto &segment : s_segments) {
            if (!segment.error_flags().is_set(SegmentError::Disconnected)) {
                ready_segment_count++;

                // Check for stale data in case of unreliable communication, for example.
                if (current_time - segment.last_update_time() >= k_schedule_tolerance) {
                    master_flags.set(MasterError::DeadlineOverrun);
                }
                if (segment.error_flags().any_set()) {
                    master_flags.set(MasterError::SegmentError);
                }
            }
        }
        xSemaphoreGive(*s_segments_mutex);

        // Check that we have the right amount of segments connected.
        if (ready_segment_count != s_config.segment_count) {
            master_flags.set(MasterError::BadSegmentCount);
        }

        // TODO: Check MCU values.
        // TODO: Check that current sensor zero voltage is within a suitable interval, which would both detect a
        //       bad/disconnect sensor and current already flowing when the BMS starts.
        // TODO: Check current values.

        // Check overcurrent threshold pins coming from the current sensors. These pins are active-low.
        if (!s_oc_n.read() || !s_oc_p.read()) {
            master_flags.set(MasterError::OvercurrentThreshold);
        }

        // The following section of code handles the shutdown assertion and delay logic. There are five categories of
        // cases the code needs to handle:
        // 1) non-immediate fault, clears after a period of k_shutdown_assert_delay
        // 2) non-immediate fault, clears within a period of k_shutdown_assert_delay (cancellation allowed)
        // 3) non-immediate fault, clears within a period of k_shutdown_assert_delay (cancellation disallowed)
        // 4) immediate fault, clears after a period of k_shutdown_assert_delay
        // 5) immediate fault, clears within a period of k_shutdown_assert_delay

        // Tentatively start a shutdown if any error flags are set.
        const bool should_shutdown = master_flags.any_set();
        if (should_shutdown && !shutdown_request_time) {
            // Start a new shutdown request.
            shutdown_request_time.emplace(xTaskGetTickCount());
            if constexpr (k_enable_debug_logs) {
                hal::swd_printf("%u: Started shutdown request with flags 0x%x\n", *shutdown_request_time,
                                master_flags.value());
            }
        } else if (!should_shutdown && !s_shutdown_time && k_allow_shutdown_cancellation) {
            // Cancel the request if it hasn't been fulfilled already.
            if constexpr (k_enable_debug_logs) {
                if (shutdown_request_time) {
                    hal::swd_printf("%u: Cancelling request made at time %u\n", xTaskGetTickCount(),
                                    *shutdown_request_time);
                }
            }
            shutdown_request_time.reset();
        }

        // Check if we should assert shutdown now either from a shutdown request whose grace period has
        // expired, or from any flags which cause immediate shutdown.
        bool should_shutdown_now = false;

        // Check if a shutdown request has reached the end of its delay period.
        should_shutdown_now |= shutdown_request_time && has_elapsed(*shutdown_request_time, k_shutdown_assert_delay);

        // Hard overcurrents are an immediate shutdown since they indicate a large short circuit or a faulty sensor.
        should_shutdown_now |= master_flags.is_set(MasterError::OvercurrentThreshold);

        // Assert shutdown if we should and haven't already.
        if (!s_shutdown_time && should_shutdown_now) {
            s_shutdown_time.emplace(xTaskGetTickCount());
            if constexpr (k_enable_debug_logs) {
                hal::swd_printf("%u: Asserted shutdown\n", *s_shutdown_time);
            }
        }

        // Keep track of the time elapsed since all faults have cleared.
        if (!fault_cleared_time && s_shutdown_time && !should_shutdown) {
            fault_cleared_time.emplace(xTaskGetTickCount());
            if constexpr (k_enable_debug_logs) {
                hal::swd_printf("%u: Fault cleared\n", *fault_cleared_time);
            }
        } else if (fault_cleared_time && should_shutdown) {
            // A fault has reappeared.
            fault_cleared_time.reset();
            if constexpr (k_enable_debug_logs) {
                hal::swd_printf("%u: Fault reappeared\n", xTaskGetTickCount());
            }
        }

        // Deassert shutdown if the previous fault has cleared and there are no current error flags.
        if (!should_shutdown && fault_cleared_time && has_elapsed(*fault_cleared_time, k_shutdown_deassert_delay)) {
            shutdown_request_time.reset();
            fault_cleared_time.reset();
            s_shutdown_time.reset();
            if constexpr (k_enable_debug_logs) {
                hal::swd_printf("%u: Deasserted shutdown\n", xTaskGetTickCount());
            }
        }

        // The shutdown pin is inverted since active-high signals no fault.
        s_shutdown.write(!s_shutdown_time.has_value());

        // Feed the watchdog and wait until next supervision period. The TPS3851 is extremely fast and only needs a 50
        // ns pulse, so we don't need any delays here. A longer pulse is fine since it is still orders of magnitude
        // before the timeout period.
        hal::gpio_reset(s_wdi);
        hal::gpio_set(s_wdi);
        xTaskDelayUntil(&last_schedule_time, pdMS_TO_TICKS(k_supervisor_period));
    }
}

void CurrentSensor::update(float voltage) {
    // TODO: Don't hardcode the delta time, it should be kept in sync with the timer period.
    constexpr float dt = 1.0f / 1000.0f;

    // Update voltage filter.
    constexpr float alpha = dt / (dt + k_current_sense_tau);
    m_filtered_voltage += alpha * (voltage - m_filtered_voltage);

    // Don't calculate current until we have an initial zero voltage.
    if (!m_has_initial_zero) {
        m_zero_voltage = m_filtered_voltage;
        if (freertos::uptime_ms() > 2000) {
            m_has_initial_zero = true;
        }
        return;
    }

    // Calculate new current.
    m_current = (m_filtered_voltage - m_zero_voltage) / k_current_sense_sensitivity;

    // Slowly tend the zero voltage if our current is zero.
    constexpr float beta = dt / (dt + k_current_sense_zero_tau);
    if (std::abs(m_current) < k_current_sense_zero_threshold) {
        m_zero_voltage += beta * (m_filtered_voltage - m_zero_voltage);
    }
}

void Segment::update(std::span<std::uint8_t> bytes) {
    // Since we decide the buffer size, we don't need to check each individual read.
    util::Stream stream(bytes);
    const auto i2c_error_count = *stream.read_be<std::uint32_t>();
    const auto thermistor_bitset = *stream.read_be<std::uint32_t>();
    const auto cell_tap_bitset = *stream.read_be<std::uint16_t>();
    const auto degraded_bitset = *stream.read_be<std::uint16_t>();

    std::array<std::uint16_t, k_max_cell_count> cell_voltages{};
    for (auto &voltage : cell_voltages) {
        voltage = *stream.read_be<std::uint16_t>();
    }

    std::array<std::int8_t, k_max_temperature_count> temperatures{};
    for (auto &temperature : temperatures) {
        temperature = *stream.read_byte();
    }

    // Compute the actual CRC.
    const auto crc = compute_crc(stream.bytes());
    const auto expected_crc = *stream.read_be<std::uint32_t>();

    xSemaphoreTake(*s_segments_mutex, portMAX_DELAY);
    if (crc != expected_crc) {
        ++m_master_error_count;
        xSemaphoreGive(*s_segments_mutex);
        return;
    }

    // Update all data in one go once we know it's valid in terms of checksum and length validity.
    for (std::size_t i = 0; i < k_max_cell_count; i++) {
        if ((cell_tap_bitset & (1u << i)) != 0) {
            m_cell_voltages[i] = cell_voltages[i];
        } else {
            m_cell_voltages[i].reset();
        }
    }
    for (std::size_t i = 0; i < k_max_temperature_count; i++) {
        if ((thermistor_bitset & (1u << i)) != 0) {
            m_temperatures[i] = temperatures[i];
        } else {
            m_temperatures[i].reset();
        }
    }
    m_slave_error_count = i2c_error_count;
    m_degraded_bitset = degraded_bitset;

    // TODO: This should take into account when the data was actually sampled by the segment which should be sent over
    // I2C.
    m_last_update_time = xTaskGetTickCount();

    // Recalculate the error flags.
    m_error_flags.clear();

    // Cell count should be exactly right.
    if (std::popcount(cell_tap_bitset) != s_config.cell_count) {
        m_error_flags.set(SegmentError::BadCellCount);
    }

    // Thermistors may be allowed to drop below the maximum.
    if (std::popcount(thermistor_bitset) < s_config.minimum_thermistor_count) {
        m_error_flags.set(SegmentError::BadThermistorCount);
    }

    // Don't accept all degraded cell voltage readings.
    if (std::popcount(m_degraded_bitset) >= s_config.cell_count) {
        m_error_flags.set(SegmentError::UnreliableMeasurement);
    }

    // Clamp config limits just in-case.
    const auto undervoltage_threshold = std::max(s_config.undervoltage_threshold, k_undervoltage_threshold_limit);
    const auto overvoltage_threshold = std::min(s_config.overvoltage_threshold, k_overvoltage_threshold_limit);
    for (const auto voltage : m_cell_voltages) {
        if (voltage) {
            if (*voltage < undervoltage_threshold) {
                m_error_flags.set(SegmentError::Undervoltage);
            }
            if (*voltage > overvoltage_threshold) {
                m_error_flags.set(SegmentError::Overvoltage);
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
                m_error_flags.set(SegmentError::Undertemperature);
            }
            if (*temperature > overtemperature_threshold) {
                m_error_flags.set(SegmentError::Overtemperature);
            }
        }
    }
    xSemaphoreGive(*s_segments_mutex);
}

bool i2c_wait(i2c::StateMachine &sm) {
    // Wait for the transaction to complete up to a timeout in case the state machine gets stuck.
    const bool timeout = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(15)) == 0;
    const auto state = sm.state();
    if (!timeout && (state == i2c::State::Idle || state == i2c::State::NoAck)) {
        return state == i2c::State::Idle;
    }
    // Reset the I2C periphral.
    sm.init();
    return false;
}

bool eeprom_read(std::uint16_t page_index, std::span<std::uint8_t> data) {
    // Write address bytes.
    const auto address = page_index * k_eeprom_page_size;
    std::array<std::uint8_t, 2> address_bytes{
        static_cast<std::uint8_t>((address >> 8u) & 0xffu),
        static_cast<std::uint8_t>(address & 0xffu),
    };
    s_i2c2_sm.start_write(k_eeprom_address, address_bytes, false);
    if (!i2c_wait(s_i2c2_sm)) {
        return false;
    }

    // Read data bytes.
    s_i2c2_sm.start_read(k_eeprom_address, data, true);
    return i2c_wait(s_i2c2_sm);
}

template <util::trivially_copyable T>
bool eeprom_read(std::uint16_t page_index, T &object) {
    auto *data = std::bit_cast<std::uint8_t *>(&object);
    return eeprom_read(page_index, std::span(data, sizeof(T)));
}

bool eeprom_write_page(std::uint16_t page_index, std::span<const std::uint8_t> page) {
    // Allow writes.
    util::ScopeGuard wc_guard([] {
        hal::gpio_set(s_eeprom_wc);
    });
    hal::gpio_reset(s_eeprom_wc);

    const auto address = page_index * k_eeprom_page_size;
    const auto to_copy = std::min(page.size(), static_cast<std::size_t>(k_eeprom_page_size));
    std::array<std::uint8_t, k_eeprom_page_size + 2> bytes{
        static_cast<std::uint8_t>((address >> 8u) & 0xffu),
        static_cast<std::uint8_t>(address & 0xffu),
    };
    std::copy_n(page.begin(), to_copy, std::next(bytes.begin(), 2));
    s_i2c2_sm.start_write(k_eeprom_address, bytes, true);
    if (!i2c_wait(s_i2c2_sm)) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    return true;
}

bool eeprom_write(std::uint16_t page_index, std::span<const std::uint8_t> data) {
    for (std::uint16_t byte_index = 0; byte_index < data.size(); byte_index += k_eeprom_page_size) {
        if (!eeprom_write_page(page_index++, data.subspan(byte_index))) {
            return false;
        }
    }
    return true;
}

template <util::trivially_copyable T>
bool eeprom_write(std::uint16_t page_index, const T &object) {
    const auto *data = std::bit_cast<const std::uint8_t *>(&object);
    return eeprom_write(page_index, std::span(data, sizeof(T)));
}

bool read_config(std::uint16_t page_index, Config &config) {
    // TODO: EEPROM failure should trigger a different error.
    if (!eeprom_read(page_index, config)) {
        return false;
    }
    if (config.magic != k_config_magic) {
        return false;
    }
    const auto *bytes = std::bit_cast<const std::uint8_t *>(&config);
    const auto crc = compute_crc(std::span(bytes, sizeof(Config) - 4));
    return crc == config.crc;
}

void config_task(void *) {
    // Initialise the I2C bus with EEPROM.
    s_i2c2_sm.init();

    // Try to read the latest valid config stored on EEPROM.
    std::optional<Config> newest_config;
    std::uint8_t config_index = 0;
    for (std::uint8_t index = 0; index < k_config_offsets.size(); index++) {
        Config config;
        if (read_config(k_config_offsets[index], config)) {
            if (!newest_config || config.counter > newest_config->counter) {
                newest_config.emplace(config);
                config_index = index;
            }
        }
    }

    // Copy config to global config.
    if (newest_config) {
        taskENTER_CRITICAL();
        s_config = *newest_config;
        taskEXIT_CRITICAL();
    }

    // Dump config.
    if constexpr (k_enable_debug_logs) {
        hal::swd_printf("\nUsing config version %u in slot %u\n", s_config.counter, config_index);
        hal::swd_printf("segment_start_address: 0x%x\n", s_config.segment_start_address);
        hal::swd_printf("segment_count: %u\n", s_config.segment_count);
        hal::swd_printf("cell_count: %u\n", s_config.cell_count);
        hal::swd_printf("minimum_thermistor_count: %u\n", s_config.minimum_thermistor_count);
        hal::swd_printf("undervoltage_threshold: %u\n", s_config.undervoltage_threshold);
        hal::swd_printf("overvoltage_threshold: %u\n", s_config.overvoltage_threshold);
        hal::swd_printf("undertemperature_threshold: %d\n", s_config.undertemperature_threshold);
        hal::swd_printf("overtemperature_threshold: %d\n", s_config.overtemperature_threshold);
        hal::swd_printf("\n");
    }

    // Install config listeners on FIFO 1.
    can::listen<WriteConfigMessage, [](const WriteConfigMessage &) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        xTaskNotifyIndexedFromISR(*s_config_task, 1, 1, eIncrement, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }>(config::k_bms_can_id, 1, 0);
    can::listen<ConfigSegmentMessage, [](const ConfigSegmentMessage &new_config) {
        s_config.segment_start_address = new_config.start_address;
        s_config.segment_count = new_config.segment_count;
        s_config.cell_count = new_config.cell_count;
        s_config.minimum_thermistor_count = new_config.minimum_thermistor_count;
    }>(config::k_bms_can_id, 1, 1);
    can::listen<ConfigThresholdMessage, [](const ConfigThresholdMessage &new_config) {
        s_config.undervoltage_threshold = new_config.undervoltage_threshold;
        s_config.overvoltage_threshold = new_config.overvoltage_threshold;
        s_config.undertemperature_threshold = new_config.undertemperature_threshold;
        s_config.overtemperature_threshold = new_config.overtemperature_threshold;
    }>(config::k_bms_can_id, 1, 2);

    while (true) {
        // Wait for a config write request.
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);

        // Use the next slot not currently in use.
        config_index = (config_index + 1) % k_config_offsets.size();
        s_config.magic = k_config_magic;
        s_config.counter++;
        if constexpr (k_enable_debug_logs) {
            hal::swd_printf("Saving config version %u to slot %u\n", s_config.counter, config_index);
        }

        // Compute new CRC.
        const auto *bytes = std::bit_cast<const std::uint8_t *>(&s_config);
        s_config.crc = compute_crc(std::span(bytes, sizeof(Config) - 4));

        // TODO: Don't ignore failure here.
        eeprom_write(k_config_offsets[config_index], s_config);
    }
}

void sample_segments_task(void *) {
    std::array<std::uint8_t, 64> buffer{};
    s_last_segment_sample_time = xTaskGetTickCount();
    while (true) {
        // Wait segment sample period.
        xTaskDelayUntil(&s_last_segment_sample_time, pdMS_TO_TICKS(k_segment_sample_period));

        // Send command to all segments via general call address.
        util::Stream stream(buffer);
        stream.write_byte(0xaa);
        stream.write_be<std::uint16_t>(0x00);
        stream.write_be<std::uint32_t>(compute_crc(stream.bytes()));
        s_i2c1_sm.start_write(0x00, stream.bytes(), true);
        if (!i2c_wait(s_i2c1_sm)) {
            // Bus failure or no acknowledge from any segments - don't even bother trying to read the segments.
            continue;
        }

        for (std::size_t index = 0; index < k_max_segment_count; index++) {
            std::fill(buffer.begin(), buffer.end(), 0);
            auto sub_buffer = std::span(buffer).subspan(0, k_segment_response_size);
            s_i2c1_sm.start_read(s_config.segment_start_address + index, sub_buffer, true);
            if (i2c_wait(s_i2c1_sm)) {
                s_segments[index].update(sub_buffer);
            }
        }
    }
}

void sample_mcu_task(void *) {
    // Sequence the LVS voltage reading, external reference voltage, and the STM's internal temperature sensor.
    hal::adc_init(ADC1, 3);
    hal::adc_sequence_channel(ADC1, 1, 1, 0b010u);
    hal::adc_sequence_channel(ADC1, 2, 7, 0b010u);
    hal::adc_sequence_channel(ADC1, 3, 16, 0b111u);

    std::array<std::uint16_t, 3> adc_buffer{};
    hal::adc_init_dma(adc_buffer);

    s_last_mcu_sample_time = xTaskGetTickCount();
    while (true) {
        // Calculate LVS input voltage. The input has a 5.3x divider.
        const auto lvs_voltage = (((k_mcu_vref * adc_buffer[0]) >> 12) * 53) / 10;

        // Calculate REF voltage. The input has a 2x divider.
        const auto ref_voltage = ((k_mcu_vref * adc_buffer[1]) >> 12) * 2;

        // Calculate an approximate temperature using constants from the datasheet.
        const auto temperature_voltage = (k_mcu_vref * adc_buffer[2]) >> 12;
        const auto temperature = ((1430 - temperature_voltage) * 10) / 43 + 25;

        // Update global values.
        xSemaphoreTake(*s_mcu_mutex, portMAX_DELAY);
        s_lvs_voltage = lvs_voltage;
        s_ref_voltage = ref_voltage;
        s_mcu_temperature = temperature;
        xSemaphoreGive(*s_mcu_mutex);

        // Start next ADC sample.
        hal::adc_start(ADC1);
        xTaskDelayUntil(&s_last_mcu_sample_time, pdMS_TO_TICKS(k_mcu_sample_period));
    }
}

void swd_task(void *) {
    TickType_t last_schedule_time = xTaskGetTickCount();
    while (true) {
        hal::swd_printf("--------------------------------\n");
        hal::swd_printf("Shutdown asserted: %s\n", !s_shutdown.read() ? "yes" : "no");
        if (s_shutdown_time) {
            const auto time_since_assertion = pdTICKS_TO_MS(xTaskGetTickCount() - *s_shutdown_time);
            hal::swd_printf("Time since shutdown assertion: %u\n", time_since_assertion);
        }
        hal::swd_printf("Uptime: %u\n", freertos::uptime_ms() / 1000);
        hal::swd_printf("CAN online: %s\n", can::is_online() ? "yes" : "no");

        const auto can_stats = can::get_stats();
        hal::swd_printf("CAN status: %u/%u %u/%u\n", can_stats.rx_count, can_stats.lost_rx_count, can_stats.tx_count,
                        can_stats.lost_tx_count);

        xSemaphoreTake(*s_mcu_mutex, portMAX_DELAY);
        hal::swd_printf("LVS voltage: %u\n", s_lvs_voltage);
        hal::swd_printf("REF voltage: %u\n", s_ref_voltage);
        hal::swd_printf("MCU temperature: %d\n", s_mcu_temperature);
        xSemaphoreGive(*s_mcu_mutex);

        const auto positive_current = static_cast<std::int32_t>(m_positive_sensor.current() * 1000.0f);
        hal::swd_printf("Positive current: %d\n", positive_current);

        const auto current_time = xTaskGetTickCount();
        xSemaphoreTake(*s_segments_mutex, portMAX_DELAY);
        for (std::size_t i = 0; i < s_segments.size(); i++) {
            const auto &segment = s_segments[i];
            if (segment.error_flags().is_set(SegmentError::Disconnected)) {
                continue;
            }

            hal::swd_printf("Segment %u (sampled %u ms ago)\n", i, current_time - segment.last_update_time());
            hal::swd_printf("  Error flags: 0x%x\n", static_cast<std::uint32_t>(segment.error_flags()));
            hal::swd_printf("  I2C error counts: %u %u\n", segment.master_error_count(), segment.slave_error_count());
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
        xSemaphoreGive(*s_segments_mutex);

        xTaskDelayUntil(&last_schedule_time, pdMS_TO_TICKS(1000));
    }
}

} // namespace

extern "C" void TIM3_IRQHandler() {
    // Clear update pending flag.
    TIM3->SR = ~TIM_SR_UIF;

    // Enable the external interrupt on MISO to capture the busy bit.
    EXTI->IMR |= EXTI_IMR_MR14;

    // Initiate an ADC conversion.
    hal::gpio_set(s_adc_cs);
    hal::gpio_reset(s_adc_cs);
}

extern "C" void EXTI15_10_IRQHandler() {
    // Clear pending flag.
    EXTI->PR = EXTI_PR_PR14;

    // Skip over the MAX11163's busy bit by clocking once manually.
    s_sck.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max10);
    hal::gpio_set(s_sck);
    hal::gpio_reset(s_sck);
    s_sck.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max10);

    // Disable the external interrupt so that it doesn't false trigger when doing the real SPI transfer.
    EXTI->IMR &= ~EXTI_IMR_MR14;

    // Transfer to the ADC.
    SPI2->DR = 0u;
}

extern "C" void SPI2_IRQHandler() {
    if ((SPI2->SR & SPI_SR_RXNE) != 0) {
        // Convert to floating point volts.
        const auto voltage = static_cast<float>((SPI2->DR * k_adc_vref) >> 16) * 0.0001f;
        m_positive_sensor.update(voltage);
    }
}

extern "C" void I2C1_EV_IRQHandler() {
    if (!s_i2c1_sm.event()) {
        // State not changed.
        return;
    }

    const auto state = s_i2c1_sm.state();
    if (state == i2c::State::Idle || state == i2c::State::NoAck || state == i2c::State::Error) {
        // Signal transaction completion.
        BaseType_t higher_priority_task_woken = pdFALSE;
        xTaskNotifyFromISR(*s_sample_segments_task, 1, eIncrement, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

extern "C" void I2C1_ER_IRQHandler() {
    s_i2c1_sm.error();
    BaseType_t higher_priority_task_woken = pdFALSE;
    xTaskNotifyFromISR(*s_sample_segments_task, 1, eIncrement, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

extern "C" void I2C2_EV_IRQHandler() {
    if (!s_i2c2_sm.event()) {
        // State not changed.
        return;
    }

    const auto state = s_i2c2_sm.state();
    if (state == i2c::State::Idle || state == i2c::State::NoAck || state == i2c::State::Error) {
        // Signal transaction completion.
        BaseType_t higher_priority_task_woken = pdFALSE;
        xTaskNotifyFromISR(*s_config_task, 1, eIncrement, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

extern "C" void I2C2_ER_IRQHandler() {
    s_i2c2_sm.error();
    BaseType_t higher_priority_task_woken = pdFALSE;
    xTaskNotifyFromISR(*s_config_task, 1, eIncrement, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

void vApplicationIdleHook() {
    // TODO: Could disable some peripherals like I2C.
    // TODO: Test power consumption of this and verify that it doesn't affect timing.
    hal::enter_sleep_mode(hal::WakeupSource::Interrupt);
}

void app_main() {
    // Configure ADC inputs.
    s_lvs_sample.configure(hal::GpioInputMode::Analog);
    s_ref_sample.configure(hal::GpioInputMode::Analog);

    // Configure watchdog input (feed) and set pins. Default WDI to high since the watchdog feeds on a falling edge.
    s_wdi.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_wds.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    hal::gpio_set(s_wdi);

    // The overcurrent pins have external pull-ups.
    s_oc_n.configure(hal::GpioInputMode::Floating);
    s_oc_p.configure(hal::GpioInputMode::Floating);

    // Shutdown output is push-pull but also has an external pull-down in case of MCU failure.
    s_shutdown.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_eeprom_wc.configure(hal::GpioOutputMode::OpenDrain, hal::GpioOutputSpeed::Max2);
    s_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

    // Default write control pin high (writes disabled).
    hal::gpio_set(s_eeprom_wc);

    // Configure I2C pins for peripheral use.
    for (const auto &pin : {s_scl_1, s_sda_1, s_scl_2, s_sda_2}) {
        pin.configure(hal::GpioOutputMode::AlternateOpenDrain, hal::GpioOutputSpeed::Max2);
    }

    // Configure current sensing and SPI pins.
    s_current_switch.configure(hal::GpioOutputMode::OpenDrain, hal::GpioOutputSpeed::Max2);
    s_adc_cs.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_miso.configure(hal::GpioInputMode::PullUp);

    // Lock pins whose configurations don't need to change.
    hal::gpio_lock(s_lvs_sample, s_ref_sample, s_adc_cs, s_current_switch, s_wdi, s_wds, s_oc_n, s_oc_p, s_shutdown,
                   s_eeprom_wc, s_led, s_miso);

    // Enable the backup domain and disable write protection.
    RCC->APB1ENR |= RCC_APB1ENR_BKPEN;
    PWR->CR |= PWR_CR_DBP;

    // Perform watchdog self-test if needed.
    if (std::exchange(BKP->DR1, 0xaaaa) != 0xaaaa) {
        // Activate the watchdog and don't feed it to force a reset.
        hal::gpio_set(s_wds, s_led);
        while (true) {
            hal::enter_sleep_mode(hal::WakeupSource::Interrupt);
        }
    }

    // Reset the backup domain and disable it.
    RCC->BDCR |= RCC_BDCR_BDRST;
    RCC->BDCR &= ~RCC_BDCR_BDRST;
    PWR->CR &= ~PWR_CR_DBP;
    RCC->APB1ENR &= ~RCC_APB1ENR_BKPEN;

    // Setup 16-bit SPI for the current sampling ADC.
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    SPI2->CR2 = SPI_CR2_RXNEIE;
    SPI2->CR1 = SPI_CR1_DFF | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE | SPI_CR1_BR_1 | SPI_CR1_MSTR;

    // Setup a falling edge external interrupt on MISO for triggering on ADC completion.
    AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI14_PB;
    EXTI->FTSR |= EXTI_FTSR_TR14;

    // Setup a 1 kHz timer to trigger current sampling.
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->DIER |= TIM_DIER_UIE;
    TIM3->PSC = 111;
    TIM3->ARR = 499;
    TIM3->CR1 |= TIM_CR1_CEN;

    s_segments_mutex.init();
    s_mcu_mutex.init();

    // Initialise all tasks.
    // TODO: Think about priorities more.
    s_supervisor_task.init(&supervisor_task, "supervisor", 4);
    s_sample_segments_task.init(&sample_segments_task, "sample_segs", 3);
    s_sample_mcu_task.init(&sample_mcu_task, "sample_mcu", 2);
    s_config_task.init(&config_task, "config", 1);
    if constexpr (k_enable_debug_logs) {
        s_swd_task.init(&swd_task, "swd", 1);
    }

    // Start the scheduler which we shouldn't return from.
    vTaskStartScheduler();
    while (true) {
        // If we somehow get here, signal shutdown and let the watchdog timeout.
        hal::gpio_reset(s_shutdown);
    }
}
