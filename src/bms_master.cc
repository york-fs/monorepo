#include <bms.hh>
#include <can.hh>
#include <eeprom.hh>
#include <hal.hh>
#include <max_adc.hh>
#include <stm32f103xb.h>
#include <util.hh>

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>

namespace {

// I2C address of the first segment.
constexpr std::uint8_t k_segment_address_start = 0x40;

// Maximum number of connected segments.
constexpr std::size_t k_max_segment_count = 12;

// M24C64-R EEPROM I2C address.
constexpr std::uint8_t k_eeprom_address = 0x50;

// Current sensor degraded thresholds in ADC counts.
// TODO: Need to experiment with this.
constexpr std::uint16_t k_sensor_degraded_threshold = 1000;

// Hard-coded value of the on-board precision voltage reference in 100 uV resolution.
constexpr std::uint16_t k_reference_voltage = 40960;

// Configuration header magic number.
constexpr std::uint32_t k_config_magic = 0x6c72d132;

struct StoredConfig {
    std::uint32_t magic;
    bms::Config config;
    std::uint32_t crc;
};

enum class CurrentSensor {
    Positive,
    Negative,
};

enum class LedState : std::uint32_t {
    Off = 0u,
    BadCan,
    BadConfig,
    BadPeripheral,

    // Special cases.
    Solid,
    SafetyShutdown,
};

class Segment {
    bms::SegmentData m_data{};
    std::uint8_t m_address{};

public:
    bool read();
    bool is_valid() const { return m_data.valid; }
    void set_address(std::uint8_t address) { m_address = address; }
    const bms::SegmentData &data() const { return m_data; }
};

hal::Gpio s_lv_reading(hal::GpioPort::A, 1);
hal::Gpio s_eeprom_wc(hal::GpioPort::A, 9);
hal::Gpio s_current_switch(hal::GpioPort::A, 10);
hal::Gpio s_oc_n(hal::GpioPort::A, 11);
hal::Gpio s_oc_p(hal::GpioPort::A, 12);
hal::Gpio s_charge_en(hal::GpioPort::B, 0);
hal::Gpio s_shutdown(hal::GpioPort::B, 1);
hal::Gpio s_led(hal::GpioPort::B, 5);

// I2C pins.
hal::Gpio s_scl_1(hal::GpioPort::B, 6);
hal::Gpio s_sda_1(hal::GpioPort::B, 7);
hal::Gpio s_scl_2(hal::GpioPort::B, 10);
hal::Gpio s_sda_2(hal::GpioPort::B, 11);

// SPI pins.
hal::Gpio s_adc_cs(hal::GpioPort::A, 8);
hal::Gpio s_sck(hal::GpioPort::B, 13);
hal::Gpio s_miso(hal::GpioPort::B, 14);

std::array<std::uint32_t, static_cast<std::uint32_t>(LedState::Solid) * 2u> s_led_dma{};
LedState s_led_state{LedState::Off};

LedState error_flags_to_led_state(bms::ErrorFlags flags) {
    // CAN error has highest priority since the error flags then can't be reported over CAN.
    if (flags.is_set(bms::Error::BadCan)) {
        return LedState::BadCan;
    }

    // Bad EEPROM or current sensor.
    if (flags.is_set(bms::Error::BadEeprom) || flags.is_set(bms::Error::BadSensor)) {
        return LedState::BadPeripheral;
    }

    // Invalid config.
    if (flags.is_set(bms::Error::BadConfig)) {
        return LedState::BadConfig;
    }

    // Otherwise the error is safety related, e.g. overvoltage.
    return LedState::SafetyShutdown;
}

bool Segment::read() {
    m_data.valid = false;

    // Bus should already be idle.
    if (hal::i2c_wait_idle(I2C1, 5) != hal::I2cStatus::Ok) {
        return false;
    }

    // Read bytes.
    std::array<std::uint8_t, sizeof(bms::SegmentData)> data_bytes{};
    if (hal::i2c_master_read(I2C1, m_address, data_bytes, 10) != hal::I2cStatus::Ok) {
        hal::i2c_stop(I2C1);
        return false;
    }
    hal::i2c_stop(I2C1);

    // Parse data.
    // TODO: Add a checksum.
    std::span<const std::uint8_t> span = data_bytes;
    m_data = {
        .thermistor_bitset = util::read_be<std::uint32_t>(span.subspan<0, 4>()),
        .cell_tap_bitset = util::read_be<std::uint16_t>(span.subspan<4, 2>()),
        .degraded_bitset = util::read_be<std::uint16_t>(span.subspan<6, 2>()),
        .rail_voltage = util::read_be<std::uint16_t>(span.subspan<8, 2>()),
        .valid = span[57] == 1,
    };
    for (std::size_t i = 0; i < m_data.voltages.size(); i++) {
        m_data.voltages[i] = util::read_be<std::uint16_t>(span.subspan(i * 2 + 10).subspan<0, 2>());
    }
    std::copy_n(&span[34], m_data.temperatures.size(), m_data.temperatures.data());
    return m_data.valid;
}

void set_led_state(LedState state) {
    if (std::exchange(s_led_state, state) == state) {
        return;
    }

    // Disable DMA channel.
    DMA1_Channel7->CCR &= ~DMA_CCR_EN;

    // Handle special solid case.
    if (state == LedState::Solid) {
        s_led_dma[0] = 1u << 5u;
        DMA1_Channel7->CNDTR = 1;
        DMA1_Channel7->CCR |= DMA_CCR_EN;
        return;
    }
    if (state == LedState::SafetyShutdown) {
        s_led_dma[0] = 1u << 5u;
        s_led_dma[1] = 1u << 21u;
        DMA1_Channel7->CNDTR = 2;
        DMA1_Channel7->CCR |= DMA_CCR_EN;
        return;
    }

    const auto count = static_cast<std::uint32_t>(state);
    for (std::uint32_t i = 0; i < count; i++) {
        s_led_dma[i * 2] = 1u << 5u;
        s_led_dma[i * 2 + 1] = 1u << 21u;
    }
    s_led_dma[count * 2] = 1u << 21u;
    s_led_dma[count * 2 + 1] = 1u << 21u;

    // Re-enable DMA channel.
    DMA1_Channel7->CNDTR = count * 2 + 2;
    DMA1_Channel7->CCR |= DMA_CCR_EN;
}

void read_config(bms::Config &config, bms::ErrorFlags &error_flags) {
    Eeprom eeprom(I2C2, s_eeprom_wc, k_eeprom_address);
    StoredConfig stored_config{};
    if (eeprom.read(0, stored_config) != hal::I2cStatus::Ok) {
        error_flags.set(bms::Error::BadConfig);
        error_flags.set(bms::Error::BadEeprom);
        return;
    }

    if (stored_config.magic != k_config_magic) {
        error_flags.set(bms::Error::BadConfig);
        return;
    }

    const auto crc = hal::crc_compute(std::span(std::bit_cast<std::uint8_t *>(&stored_config), sizeof(StoredConfig)));
    if (crc != 0) {
        error_flags.set(bms::Error::BadConfig);
        return;
    }
    config = stored_config.config;
}

bool write_config(const bms::Config &config) {
    Eeprom eeprom(I2C2, s_eeprom_wc, k_eeprom_address);
    StoredConfig stored_config{
        .magic = k_config_magic,
        .config = config,
    };

    // Compute the CRC.
    std::span config_span(std::bit_cast<std::uint8_t *>(&stored_config), sizeof(StoredConfig) - sizeof(std::uint32_t));
    stored_config.crc = hal::crc_compute(config_span);

    // Write the config starting in the first page.
    return eeprom.write(0, stored_config) == hal::I2cStatus::Ok;
}

std::optional<std::pair<std::uint16_t, std::uint32_t>> sample_current(CurrentSensor sensor,
                                                                      std::uint16_t zero_voltage) {
    if (sensor == CurrentSensor::Positive) {
        hal::gpio_reset(s_current_switch);
    } else {
        hal::gpio_set(s_current_switch);
    }

    // Wait for settle.
    // TODO: Lower this when RC is lowered and pull-down is added.
    hal::delay_us(10000);

    auto sample = max_adc::sample_voltage(SPI2, s_adc_cs, k_reference_voltage, 32);
    if (!sample) {
        // Failed to sample ADC.
        return std::nullopt;
    }
    // if (sample->second > k_sensor_degraded_threshold) {
    //     // Sample too noisy.
    //     return std::nullopt;
    // }

    // TODO: We need to flag voltages above the reference 2.5 volts as bad (backwards sensor).
    const auto voltage = std::min(sample->first, zero_voltage);
    const auto difference = static_cast<std::uint32_t>(zero_voltage - voltage);
    const auto current = (difference * 1000u) >> 5;
    return std::make_pair(voltage, current);
}

std::uint32_t sample_segments(std::span<Segment> segments) {
    // Wakeup segments by effectively pulling SCL low.
    I2C1->CR1 &= ~I2C_CR1_PE;
    s_scl_1.configure(hal::GpioOutputMode::OpenDrain, hal::GpioOutputSpeed::Max2);
    s_scl_1.configure(hal::GpioOutputMode::AlternateOpenDrain, hal::GpioOutputSpeed::Max2);
    hal::delay_us(200);

    // Reinitialise the I2C peripheral.
    hal::i2c_init(I2C1, std::nullopt);

    std::uint32_t segment_count = 0;
    for (auto &segment : segments) {
        if (segment.read()) {
            segment_count++;
        }
    }
    return segment_count;
}

} // namespace

void app_main() {
    // Configure LED and 12 volt rail measurement pin.
    s_lv_reading.configure(hal::GpioInputMode::Analog);
    s_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

    // These outputs are open-drain as they require 5 volts logic high, and so have external pull-ups.
    s_eeprom_wc.configure(hal::GpioOutputMode::OpenDrain, hal::GpioOutputSpeed::Max2);
    s_current_switch.configure(hal::GpioOutputMode::OpenDrain, hal::GpioOutputSpeed::Max2);

    // Configure overcurrent inputs as floating as they have external pull-ups.
    s_oc_n.configure(hal::GpioInputMode::Floating);
    s_oc_p.configure(hal::GpioInputMode::Floating);

    // Charge enable and shutdown are push-pull. They have external pull-downs in case the MCU has a problem.
    s_charge_en.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_shutdown.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

    // Configure I2C pins.
    for (const auto &pin : {s_scl_1, s_sda_1, s_scl_2, s_sda_2}) {
        pin.configure(hal::GpioOutputMode::AlternateOpenDrain, hal::GpioOutputSpeed::Max2);
    }

    // Configure ADC SPI pins. Enable a pull-up on MISO to avoid floating when no slave is selected.
    s_adc_cs.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_sck.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max10);
    s_miso.configure(hal::GpioInputMode::PullUp);

    // Default open drain pins to high impedance state.
    hal::gpio_set(s_eeprom_wc, s_current_switch);

    // Startup delay in case of PVD or watchdog trigger.
    hal::delay_us(100000);

    // Initialise both I2C buses and SPI for the ADC.
    hal::i2c_init(I2C1, std::nullopt);
    hal::i2c_init(I2C2, std::nullopt);
    hal::spi_init_master(SPI2, SPI_CR1_BR_2);

    // Initialise the ADC.
    hal::adc_init(ADC1, 1);
    hal::adc_sequence_channel(ADC1, 1, 16, 0b111u);

    const float V_25 = 1.43f;
    const float avg_slope = 0.0043f;

    while (true) {
        const std::uint32_t voltage = ADC1->DR & 0xffffu;
        float V_sense = (voltage / 4095.0f) * 3.3f;
        float temperature = (V_25 - V_sense) / avg_slope + 25.0f;
        
        hal::swd_printf("ADC data: %u %u %u\n", voltage, (voltage * 3300) >> 12, (uint32_t) temperature);
        hal::adc_start(ADC1);
        hal::delay_us(500000);
    }

    // Set segment addresses.
    std::array<Segment, k_max_segment_count> segments;
    for (std::uint8_t i = 0; i < segments.size(); i++) {
        segments[i].set_address(k_segment_address_start + i);
    }

    // Setup timer for LED DMA.
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->DIER |= TIM_DIER_UDE;
    TIM4->PSC = 1999;
    TIM4->ARR = 3999;
    TIM4->CR1 |= TIM_CR1_CEN;
    DMA1_Channel7->CPAR = std::bit_cast<std::uint32_t>(&GPIOB->BSRR);
    DMA1_Channel7->CMAR = std::bit_cast<std::uint32_t>(s_led_dma.data());
    DMA1_Channel7->CCR = DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_DIR;

    // State variables.
    bms::Config config{};
    bms::ErrorFlags error_flags;

    // Attempt to initialise the CAN peripheral on PB8 (RX) and PB9 (TX).
    if (!can::init(can::Port::B, can::Speed::_500)) {
        error_flags.set(bms::Error::BadCan);
    }

    // Try to read the config from the EEPROM.
    read_config(config, error_flags);

    // Warm up ADC.
    for (std::size_t i = 0; i < 256; i++) {
        static_cast<void>(max_adc::sample_raw(SPI2, s_adc_cs));
    }

    // Read current sensor idle reference voltages.
    // TODO: Ensure within absolute tolerance of expected 2.5 volts.
    const auto positive_sensor_zero_sample = sample_current(CurrentSensor::Positive, UINT16_MAX);
    const auto negative_sensor_zero_sample = sample_current(CurrentSensor::Negative, UINT16_MAX);
    if (!positive_sensor_zero_sample || !negative_sensor_zero_sample) {
        error_flags.set(bms::Error::BadSensor);
    }
    const auto positive_sensor_zero_voltage = positive_sensor_zero_sample.value_or(std::make_pair(0, 0)).first;
    const auto negative_sensor_zero_voltage = negative_sensor_zero_sample.value_or(std::make_pair(0, 0)).first;
    hal::swd_printf("CUR+ zero voltage: %u\n", positive_sensor_zero_voltage);
    hal::swd_printf("CUR- zero voltage: %u\n", negative_sensor_zero_voltage);

    // TODO: Use a timer to record how old the segment data is, need to react within 250 ms.

    // Main state machine loop.
    while (true) {
        // Check if shutdown needed.
        if (error_flags.any_set()) {
            // Disable charging and activate shutdown (active-low).
            hal::gpio_reset(s_charge_en, s_shutdown);

            // Set the onboard LED state based on the error flags.
            set_led_state(error_flags_to_led_state(error_flags));

            // Fallthrough so that sampling still takes place. We've done the important thing of activating shutdown.
        }

        // Don't continue with a bad config.
        // TODO: Allow updating the config over CAN.
        if (error_flags.is_set(bms::Error::BadConfig)) {
            continue;
        }

        if (!error_flags.any_set()) {
            // Everything normal.
            set_led_state(LedState::Solid);
        }

        // TODO: Check segment count against a "locked count" after a drive enable CAN command.
        const auto segment_count = sample_segments(segments);

        for (std::uint32_t i = 0; i < segments.size(); i++) {
            const auto &segment = segments[i];
            if (!segment.is_valid()) {
                continue;
            }

            const auto &data = segment.data();
            const auto [min_voltage, max_voltage] = bms::min_max_voltage(data);
            const auto [min_temperature, max_temperature] = bms::min_max_temperature(data);
            hal::swd_printf("Segment %u: [%u, %u] [%d, %d] 0x%x 0x%x 0x%x \n", i, min_voltage, max_voltage,
                            min_temperature, max_temperature, data.thermistor_bitset, data.cell_tap_bitset,
                            data.degraded_bitset);

            const auto segment_flags = bms::check_segment(config, data);
            error_flags.set_all(segment_flags);
        }

        // Sample current sensors.
        const auto positive_sensor_sample = sample_current(CurrentSensor::Positive, positive_sensor_zero_voltage);
        const auto negative_sensor_sample = sample_current(CurrentSensor::Negative, negative_sensor_zero_voltage);
        if (!positive_sensor_sample || !negative_sensor_sample) {
            const bool new_error = !error_flags.any_set();
            error_flags.set(bms::Error::BadSensor);
            if (new_error) {
                continue;
            }
        } else {
            hal::swd_printf("CUR+: %u %u\n", positive_sensor_sample->first, positive_sensor_sample->second);
            hal::swd_printf("CUR-: %u %u\n", negative_sensor_sample->first, negative_sensor_sample->second);
        }
    }
}
