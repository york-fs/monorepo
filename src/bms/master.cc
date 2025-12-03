#include <bms.hh>
#include <bms/error.hh>
#include <hal.hh>
#include <stm32f103xb.h>
#include <util.hh>

#include <FreeRTOS.h>
#include <task.h>

#include <algorithm>
#include <cstdint>

namespace {

/**
 * @brief The I2C address of the first segment.
 */
constexpr std::uint8_t k_segment_address_start = 0x40;

/**
 * @brief The maximum number of connected segments.
 */
constexpr std::size_t k_max_segment_count = 12;

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

hal::Gpio s_shutdown(hal::GpioPort::B, 1);
hal::Gpio s_led(hal::GpioPort::B, 5);

// I2C pins.
hal::Gpio s_scl_1(hal::GpioPort::B, 6);
hal::Gpio s_sda_1(hal::GpioPort::B, 7);
hal::Gpio s_scl_2(hal::GpioPort::B, 10);
hal::Gpio s_sda_2(hal::GpioPort::B, 11);

TickType_t s_last_segment_sample_time = 0;

/**
 * @brief The supervisor task for the BMS which is responsible for the shutdown output and watchdog feeding.
 *
 * This task has the highest priority and oversees deadlines for all of the other tasks. If any other task stops being
 * scheduled (and therefore stops monitoring the battery properly), this task will notice and open the shutdown output.
 * If this task stops being scheduled, or the whole MCU locks up, this task will stop feeding the watchdog which will
 * cause it to timeout, open the shutdown output, and reset the MCU.
 */
void supervisor_task(void *) {
    TickType_t last_schedule_time = xTaskGetTickCount();
    std::optional<TickType_t> shutdown_request_time;
    while (true) {
        // Calculate the master flags based on master state and segment data.
        bms::ErrorFlags master_flags;

        // Check for missed deadlines for all of the other tasks.
        const auto current_time = xTaskGetTickCount();
        if (current_time - s_last_segment_sample_time >= k_schedule_tolerance) {
            master_flags.set(bms::Error::DeadlineOverrun);
        }

        // Tentatively start a shutdown if any error flags are set.
        const bool should_shutdown = master_flags.any_set();
        if (should_shutdown && !shutdown_request_time) {
            // New shutdown request.
            shutdown_request_time.emplace(xTaskGetTickCount());
        } else if (!should_shutdown && k_allow_shutdown_cancellation) {
            // Cancel the request.
            shutdown_request_time.reset();
        }

        // Actually open the shutdown circuit when the grace period is over. The shutdown state can currently not be left.
        if (shutdown_request_time && xTaskGetTickCount() - *shutdown_request_time >= pdMS_TO_TICKS(k_shutdown_delay)) {
            hal::gpio_reset(s_shutdown);
        }

        xTaskDelayUntil(&last_schedule_time, pdMS_TO_TICKS(k_supervisor_period));
    }
}

bool sample_segment(bms::SegmentData &data, std::uint8_t address) {
    // Bus should already be idle.
    if (hal::i2c_wait_idle(I2C1, 5) != hal::I2cStatus::Ok) {
        return false;
    }

    // Read bytes.
    // TODO: Use interrupts.
    std::array<std::uint8_t, sizeof(bms::SegmentData)> data_bytes{};
    if (hal::i2c_master_read(I2C1, address, data_bytes, 10) != hal::I2cStatus::Ok) {
        hal::i2c_stop(I2C1);
        return false;
    }
    hal::i2c_stop(I2C1);

    // Parse data.
    // TODO: Add a checksum.
    std::span<const std::uint8_t> span = data_bytes;
    data = {
        .thermistor_bitset = util::read_be<std::uint32_t>(span.subspan<0, 4>()),
        .cell_tap_bitset = util::read_be<std::uint16_t>(span.subspan<4, 2>()),
        .degraded_bitset = util::read_be<std::uint16_t>(span.subspan<6, 2>()),
        .rail_voltage = util::read_be<std::uint16_t>(span.subspan<8, 2>()),
        .valid = span[57] == 1,
    };
    for (std::size_t i = 0; i < data.voltages.size(); i++) {
        data.voltages[i] = util::read_be<std::uint16_t>(span.subspan(i * 2 + 10).subspan<0, 2>());
    }
    std::copy_n(&span[34], data.temperatures.size(), data.temperatures.data());
    return data.valid;
}

void sample_segments_task(void *) {
    s_last_segment_sample_time = xTaskGetTickCount();
    while (true) {
        TickType_t time_start = xTaskGetTickCount();

        // Wakeup all segments by effectively pulling SCL low.
        I2C1->CR1 &= ~I2C_CR1_PE;
        s_scl_1.configure(hal::GpioOutputMode::OpenDrain, hal::GpioOutputSpeed::Max2);
        s_scl_1.configure(hal::GpioOutputMode::AlternateOpenDrain, hal::GpioOutputSpeed::Max2);
        vTaskDelay(pdMS_TO_TICKS(1));

        // Reinitialise the I2C peripheral.
        hal::i2c_init(I2C1, std::nullopt);

        for (std::uint8_t address = k_segment_address_start; address < k_segment_address_start + k_max_segment_count;
             address++) {
            bms::SegmentData data{};
            if (sample_segment(data, address)) {
                const auto [min_voltage, max_voltage] = bms::min_max_voltage(data);
                hal::swd_printf("Found segment at %u: [%u, %u]\n", address, min_voltage, max_voltage);
            }
        }

        TickType_t time_taken = xTaskGetTickCount() - time_start;
        hal::swd_printf("took %u ms (%u ticks)\n", pdTICKS_TO_MS(time_taken), time_taken);

        xTaskDelayUntil(&s_last_segment_sample_time, pdMS_TO_TICKS(50));
    }
}

} // namespace

void app_main() {
    s_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

    // Configure I2C pins for peripheral use.
    for (const auto &pin : {s_scl_1, s_sda_1, s_scl_2, s_sda_2}) {
        pin.configure(hal::GpioOutputMode::AlternateOpenDrain, hal::GpioOutputSpeed::Max2);
    }

    // Shutdown output is push-pull but also has an external pull-down in case of MCU failure.
    s_shutdown.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

    // Initialise both I2C buses and SPI for the ADC.
    hal::i2c_init(I2C1, std::nullopt);
    hal::i2c_init(I2C2, std::nullopt);
    hal::spi_init_master(SPI2, SPI_CR1_BR_2);

    // Todo: Configure OC_P and OC_N pins as interrupts for immediate shutdown.
    // TODO: Lock GPIO configurations.

    xTaskCreate(&supervisor_task, "supervisor", 256, nullptr, 2, nullptr);
    xTaskCreate(&sample_segments_task, "sample_segs", 512, nullptr, 1, nullptr);
    vTaskStartScheduler();
    while (true) {
        // If we somehow get here, signal shutdown and let the watchdog timeout.
        hal::gpio_reset(s_shutdown);
    }
}
