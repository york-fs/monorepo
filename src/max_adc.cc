#include <max_adc.hh>

#include <hal.hh>

#include <cstdint>
#include <limits>

namespace max_adc {

std::optional<std::uint16_t> sample_raw(SPI_TypeDef *spi, const hal::Gpio &chip_select) {
    // Trigger conversion.
    hal::gpio_reset(chip_select);
    hal::gpio_set(chip_select);

    // Wait maximum conversion time.
    hal::delay_us(3);

    // Read value over SPI.
    std::array<std::uint8_t, 2> bytes{};
    if (!hal::spi_transfer(spi, chip_select, bytes, 1)) {
        return std::nullopt;
    }

    // Return assembled value.
    return (static_cast<std::uint16_t>(bytes[0]) << 8u) | bytes[1];
}

std::optional<std::pair<std::uint16_t, std::uint16_t>> sample_voltage(SPI_TypeDef *spi, const hal::Gpio &chip_select,
                                                                      std::uint16_t reference_voltage,
                                                                      std::size_t sample_count) {
    std::uint16_t min_value = std::numeric_limits<std::uint16_t>::max();
    std::uint16_t max_value = 0;
    std::uint32_t sum = 0;
    for (std::size_t i = 0; i < sample_count; i++) {
        const auto value = sample_raw(spi, chip_select);
        if (!value) {
            return std::nullopt;
        }
        min_value = std::min(min_value, *value);
        max_value = std::max(max_value, *value);
        sum += *value;
    }

    const auto average = sum / sample_count;
    const auto voltage = (average * reference_voltage) >> 16u;
    return std::make_pair(voltage, max_value - min_value);
}

} // namespace max_adc
