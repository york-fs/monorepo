#pragma once

#include <hal.hh>
#include <stm32f103xb.h>
#include <util.hh>

#include <bit>
#include <cstdint>
#include <span>

/// A class for the M24C64-R EEPROM.
struct Eeprom {
    I2C_TypeDef *m_i2c;
    hal::Gpio m_write_control;
    std::uint8_t m_address;

public:
    /**
     * Constructs an EEPROM interface.
     *
     * @param i2c the i2c peripheral the EEPROM is connected to
     * @param write_control the GPIO pin controlling the EEPROM's write protection
     * @param address the 7-bit I2C address of the EEPROM
     */
    Eeprom(I2C_TypeDef *i2c, hal::Gpio write_control, std::uint8_t address)
        : m_i2c(i2c), m_write_control(write_control), m_address(address) {}

    /**
     * Reads raw bytes from the EEPROM starting at the given page index.
     *
     * @param page_index the starting page index
     * @param data the output buffer to read the bytes into
     * @return hal::I2cStatus::Ok on success; an error status otherwise
     */
    hal::I2cStatus read(std::uint16_t page_index, std::span<std::uint8_t> data);

    /**
     * Reads a typed object from the EEPROM.
     *
     * @tparam T a trivially-copyable POD type
     * @param page_index the starting page index
     * @param object a reference to the object to populate
     * @return hal::I2cStatus::Ok on success; an error status otherwise
     */
    template <util::trivially_copyable T>
    hal::I2cStatus read(std::uint16_t page_index, T &object);

    /**
     * Writes a single 32-byte page to the EEPROM. The data will be truncated if it exceeds 32 bytes.
     *
     * @param index the page index
     * @param page up to 32 bytes of data
     * @return hal::I2cStatus::Ok on success; an error status otherwise
     */
    hal::I2cStatus write_page(std::uint16_t index, std::span<const std::uint8_t> page);

    /**
     * Writes arbitrary length data to the EEPROM.
     *
     * @param page_index the starting page index
     * @param data the data buffer to write
     * @return hal::I2cStatus::Ok on success; an error status otherwise
     */
    hal::I2cStatus write(std::uint16_t page_index, std::span<const std::uint8_t> data);

    /**
     * Writes a typed object to the EEPROM.
     *
     * @param T a trivially-copyable POD type
     * @param page_index the starting page index
     * @param object a reference to the object to write
     * @return hal::I2cStatus::Ok on success; an error status otherwise
     */
    template <util::trivially_copyable T>
    hal::I2cStatus write(std::uint16_t page_index, const T &object);
};

template <util::trivially_copyable T>
hal::I2cStatus Eeprom::read(std::uint16_t page_index, T &object) {
    auto *data = std::bit_cast<std::uint8_t *>(&object);
    return read(page_index, std::span(data, sizeof(T)));
}

template <util::trivially_copyable T>
hal::I2cStatus Eeprom::write(std::uint16_t page_index, const T &object) {
    const auto *data = std::bit_cast<const std::uint8_t *>(&object);
    return write(page_index, std::span(data, sizeof(T)));
}
