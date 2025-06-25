#include <eeprom.hh>

#include <hal.hh>
#include <util.hh>

#include <algorithm>
#include <array>
#include <cstdint>

namespace {

constexpr std::uint16_t k_page_size = 32;

} // namespace

hal::I2cStatus Eeprom::read(std::uint16_t page_index, std::span<std::uint8_t> data) {
    const auto address = page_index * k_page_size;
    std::array<std::uint8_t, 2> address_bytes{
        static_cast<std::uint8_t>((address >> 8u) & 0xffu),
        static_cast<std::uint8_t>(address & 0xffu),
    };
    if (auto status = hal::i2c_wait_idle(m_i2c); status != hal::I2cStatus::Ok) {
        return status;
    }
    if (auto status = hal::i2c_master_write(m_i2c, m_address, address_bytes); status != hal::I2cStatus::Ok) {
        return status;
    }
    if (auto status = hal::i2c_master_read(m_i2c, m_address, data, 1); status != hal::I2cStatus::Ok) {
        return status;
    }
    hal::i2c_stop(m_i2c);
    return hal::I2cStatus::Ok;
}

hal::I2cStatus Eeprom::write_page(std::uint16_t index, std::span<const std::uint8_t> page) {
    // Allow writes.
    // TODO: This needs testing on hardware revision B.
    util::ScopeGuard wc_guard([this] {
        hal::gpio_set(m_write_control);
    });
    hal::gpio_reset(m_write_control);

    const auto address = index * k_page_size;
    const auto size = std::min(page.size(), 32u);
    std::array<std::uint8_t, 34> bytes{
        static_cast<std::uint8_t>((address >> 8u) & 0xffu),
        static_cast<std::uint8_t>(address & 0xffu),
    };
    std::copy_n(page.begin(), size, std::next(bytes.begin(), 2));

    const auto span = std::span(bytes).subspan(0, size + 2);
    if (auto status = hal::i2c_wait_idle(m_i2c); status != hal::I2cStatus::Ok) {
        return status;
    }
    if (auto status = hal::i2c_master_write(m_i2c, m_address, span); status != hal::I2cStatus::Ok) {
        return status;
    }
    hal::i2c_stop(m_i2c);

    // Wait worst case write time.
    // TODO: Can poll for ACK on sending address.
    hal::delay_us(5000);
    return hal::I2cStatus::Ok;
}

hal::I2cStatus Eeprom::write(std::uint16_t page_index, std::span<const std::uint8_t> data) {
    for (std::uint16_t byte_index = 0; byte_index < data.size(); byte_index += k_page_size) {
        if (auto status = write_page(page_index++, data.subspan(byte_index)); status != hal::I2cStatus::Ok) {
            return status;
        }
    }
    return hal::I2cStatus::Ok;
}
