#pragma once

#include <hal.hh>
#include <stm32f103xb.h>

#include <cstdint>
#include <optional>
#include <utility>

namespace max_adc {

std::optional<std::uint16_t> sample_raw(SPI_TypeDef *spi, const hal::Gpio &chip_select);
std::optional<std::pair<std::uint16_t, std::uint16_t>> sample_voltage(SPI_TypeDef *spi, const hal::Gpio &chip_select,
                                                                      std::uint16_t reference_voltage,
                                                                      std::size_t sample_count);

} // namespace max_adc
