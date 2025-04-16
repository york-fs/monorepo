#include <hal.hh>
#include <stm32f103xb.h>

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

namespace {

// MAX14920 product and die version bits.
constexpr std::uint8_t k_afe_version_bits = 0b1010;

// Number of ADC samples to perform for voltage measurement.
constexpr std::uint32_t k_adc_sample_count = 8;

// Maximum number of connected thermistors, including the onboard ones.
constexpr std::uint32_t k_max_thermistor_count = 23;

// Voltage threshold from the absolute endpoints (0 and Vref) in 100 uV resolution from when to consider a thermistor as
// being either open or short circuit.
constexpr std::uint32_t k_thermistor_range_threshold = 5000;

enum class AfeStatus {
    Ready,
    NotReady,
    BadSpi,
    Shutdown,
};

class CsGuard {
    hal::Gpio m_pin;

public:
    explicit CsGuard(hal::Gpio pin) : m_pin(pin) {
        // Pull CS low.
        hal::gpio_reset(pin);
    }
    CsGuard(const CsGuard &) = delete;
    CsGuard(CsGuard &&) = delete;
    ~CsGuard() {
        // Pull CS high again.
        hal::gpio_set(m_pin);
    }

    CsGuard &operator=(const CsGuard &) = delete;
    CsGuard &operator=(CsGuard &&) = delete;
};

std::array s_thermistor_enable{
    hal::Gpio(hal::GpioPort::A, 0), hal::Gpio(hal::GpioPort::A, 1), hal::Gpio(hal::GpioPort::A, 2),
    hal::Gpio(hal::GpioPort::A, 3), hal::Gpio(hal::GpioPort::A, 4), hal::Gpio(hal::GpioPort::A, 5),
    hal::Gpio(hal::GpioPort::A, 6), hal::Gpio(hal::GpioPort::A, 7),
};
static_assert(s_thermistor_enable.size() * 3 >= k_max_thermistor_count);

hal::Gpio s_ref_en(hal::GpioPort::B, 1);
hal::Gpio s_led(hal::GpioPort::B, 5);
hal::Gpio s_scl(hal::GpioPort::B, 6);
hal::Gpio s_sda(hal::GpioPort::B, 7);
hal::Gpio s_adc_cs(hal::GpioPort::B, 9);
hal::Gpio s_afe_cs(hal::GpioPort::B, 10);
hal::Gpio s_afe_en(hal::GpioPort::B, 11);
hal::Gpio s_sck(hal::GpioPort::B, 13);
hal::Gpio s_miso(hal::GpioPort::B, 14);
hal::Gpio s_mosi(hal::GpioPort::B, 15);

void spi_transfer(const hal::Gpio &cs, std::span<std::uint8_t> data) {
    // TODO: Add timeouts.
    CsGuard cs_guard(cs);
    for (auto &byte : data) {
        // Transmit byte.
        hal::wait_equal(SPI2->SR, SPI_SR_TXE, SPI_SR_TXE);
        SPI2->DR = byte;

        // Receive byte.
        hal::wait_equal(SPI2->SR, SPI_SR_RXNE, SPI_SR_RXNE);
        byte = SPI2->DR;
    }

    // Wait for busy to be clear and then reset CS to high.
    hal::wait_equal(SPI2->SR, SPI_SR_BSY, 0u);
}

std::uint16_t adc_sample_raw() {
    // Trigger conversion.
    hal::gpio_reset(s_adc_cs);
    hal::gpio_set(s_adc_cs);

    // Read value over SPI.
    std::array<std::uint8_t, 2> bytes{};
    spi_transfer(s_adc_cs, bytes);

    // Return assembled value.
    return (static_cast<std::uint16_t>(bytes[0]) << 8u) | bytes[1];
}

std::pair<std::uint16_t, std::uint16_t> adc_sample_voltage(std::uint32_t sample_count) {
    std::uint16_t min_value = std::numeric_limits<std::uint16_t>::max();
    std::uint16_t max_value = 0;
    std::uint32_t sum = 0;
    for (std::uint32_t i = 0; i < sample_count; i++) {
        const auto value = adc_sample_raw();
        min_value = std::min(min_value, value);
        max_value = std::max(max_value, value);
        sum += value;
    }

    const auto average = sum / sample_count;
    const auto voltage = (average * 45000u) >> 16u;
    return std::make_pair(voltage, max_value - min_value);
}

AfeStatus afe_command(std::uint16_t balance_bits, std::uint8_t control_bits) {
    std::array<std::uint8_t, 3> data{
        static_cast<std::uint8_t>(balance_bits >> 8u),
        static_cast<std::uint8_t>(balance_bits),
        control_bits,
    };
    spi_transfer(s_afe_cs, data);

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

void sample_cell_voltage(std::uint8_t index) {
    constexpr std::array<std::uint8_t, 12> index_table{
        0b10000000, 0b11000000, 0b10100000, 0b11100000, 0b10010000, 0b11010000,
        0b10110000, 0b11110000, 0b10001000, 0b11001000, 0b10101000, 0b11101000,
    };

    // Deselect hold mode.
    afe_command(0u, 0b10000000u);

    // Select cell for output on AOUT.
    afe_command(0u, index_table[index] | 0b100u);

    // Wait for level shift and AOUT settle (50 us).
    // TODO: Measure this.
    for (std::uint32_t i = 0; i < 50; i++) {
        __NOP();
    }

    const auto [voltage, adc_range] = adc_sample_voltage(k_adc_sample_count);
    hal::swd_printf("v%u: [%u, %u]\n", index + 1, voltage, adc_range);
}

void sample_thermistors(std::uint16_t reference_voltage) {
    std::uint32_t bitset = 0;
    std::array<std::uint8_t, 23> temperatures;
    for (std::uint32_t index = 0; index < k_max_thermistor_count; index++) {
        // Enable the MOSFET.
        GPIOA->ODR = 1u << (index / 3);

        const auto selection_bits = std::array<std::uint8_t, 3>{
            0b01011000u, // T1 (buffered)
            0b00111000u, // T2 (buffered)
            0b01111000u, // T3 (buffered)
        }[index % 3];
        afe_command(0u, selection_bits);

        // Wait for settle (5 us).
        for (std::uint32_t i = 0; i < 10; i++) {
            __NOP();
        }

        const auto adc_value = adc_sample_raw();
        const auto voltage = (adc_value * 45000u) >> 16u;
        if (std::abs(static_cast<std::int32_t>(voltage) - reference_voltage) > k_thermistor_range_threshold) {
            // Thermistor is connected.
            bitset |= 1u << index;

            const auto resistance = (voltage * 10000) / (reference_voltage - voltage);
            float beta = 1.0f / 3950.0f;
            if (index < 3) {
                beta = 1.0f / 3350.0f;
            }
            float t0 = 1.0f / 298.15f;
            float temperature = 1.0f / (t0 + beta * std::log(static_cast<float>(resistance) / 10000.0f)) - 273.15f;
            temperatures[index] = static_cast<std::uint32_t>(temperature);
        }
    }

    // Disable all MOSFETs.
    GPIOA->ODR = 0u;

    hal::swd_printf("%u thermistors connected\n", std::popcount(bitset));
    for (std::uint32_t index = 0; index < k_max_thermistor_count; index++) {
        if ((bitset & (1u << index)) != 0u) {
            hal::swd_printf("t%u: %u\n", index, temperatures[index]);
        }
    }
}

} // namespace

bool hal_low_power() {
    // Use the 8 MHz internal clock.
    return true;
}

void app_main() {
    // Configure general GPIOs.
    for (const auto &gpio : s_thermistor_enable) {
        gpio.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    }
    s_ref_en.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_afe_en.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

    // Configure CS pin and enable a pull-up on MISO to avoid floating when no slave is selected.
    s_adc_cs.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_afe_cs.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_miso.configure(hal::GpioInputMode::PullUp);

    // Enable clocks for I2C1 and SPI2.
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN | RCC_APB1ENR_SPI2EN;

    // Enable external interrupt on SCL (PB6).
    AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI6_PB;
    EXTI->EMR |= EXTI_EMR_MR6;
    EXTI->FTSR |= EXTI_FTSR_TR6;

    while (true) {
        // Reconfigure SCK and MOSI as regular GPIOs before going to sleep.
        s_sck.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
        s_mosi.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

        // Pull CS lines high by default (active-low) and put the ADC, AFE, and reference into shutdown.
        hal::gpio_set(s_adc_cs, s_afe_cs, s_sck);
        hal::gpio_reset(s_adc_cs, s_afe_en, s_ref_en);
        hal::gpio_set(s_adc_cs);

        // Reconfigure SCL as a regular input for use as an external event and enter stop mode.
        s_scl.configure(hal::GpioInputMode::Floating);
        hal::enter_stop_mode();

        // Wake the ADC and enable the AFE and reference.
        hal::gpio_reset(s_sck, s_adc_cs);
        hal::gpio_set(s_adc_cs, s_afe_cs, s_afe_en, s_ref_en);

        // Configure SCK and MOSI for use with SPI peripheral.
        s_sck.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max10);
        s_mosi.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max10);

        // Enable SPI2 in master mode at 2 MHz (4x divider).
        SPI2->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE | SPI_CR1_BR_0 | SPI_CR1_MSTR;

        // Wait for AFE startup to complete.
        while (afe_command(0, 0) == AfeStatus::NotReady) {
            __NOP();
        }

        // Wait 100 ms for sampling and reference comeup.
        for (std::uint32_t i = 0; i < 400000; i++) {
            __NOP();
        }

        // Route T1 (buffered) with all thermistors switched off to measure the 3V3 voltage.
        afe_command(0u, 0b01011000u);
        const auto [rail_voltage, rail_noise] = adc_sample_voltage(128);
        hal::swd_printf("3v3 rail: [%u, %u]\n", rail_voltage, rail_noise);

        // Sample all thermistors.
        sample_thermistors(rail_voltage);

        // Sample all cells.
        for (std::uint32_t cell = 12; cell > 0; cell--) {
            sample_cell_voltage(cell - 1);
        }
    }
}
