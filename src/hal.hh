#pragma once

#include <stm32f103xb.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <utility>

namespace hal {

/**
 * @brief The result of an I2C operation.
 */
enum class [[nodiscard]] I2cStatus {
    /**
     * @brief General OK status.
     */
    Ok,

    /**
     * @brief Returned by slave functions to indicate that the master wants to send data to the slave.
     */
    OkRead,

    /**
     * @brief Returned by slave functions to indicate that the master wants to receive data from the slave.
     */
    OkWrite,

    /**
     * @brief Returned by master functions if no slave acknowledged an address in time, or if a slave fails to
     * acknowledge transmitted bytes in time.
     */
    AcknowledgeFailure,

    /**
     * @brief General timeout error.
     */
    Timeout,
};

enum class WakeupSource {
    Interrupt,
    Event,
};

/**
 * @brief Unmasks the given IRQ in the NVIC interrupt controller with the given priority. A lower priority number result
 * in a higher priority, i.e. 0 is the highest priority.
 *
 * @param irq the IRQ to unmask
 * @param priority the priority to set; must be in the range [0, 15]
 */
void irq_enable(IRQn_Type irq, std::uint32_t priority);

/**
 * @brief Remasks the given IRQ in the NVIC interrupt controller.
 *
 * @param irq the IRQ to mask
 */
void irq_disable(IRQn_Type irq);

/**
 * @brief Places the MCU into sleep mode. In this mode, only the core clock is stopped. The MCU will wake up on either
 * an interrupt or an event, depending on the passed parameter.
 *
 * @param source whether to wakeup on an interrupt or an event
 */
void enter_sleep_mode(WakeupSource source);

/**
 * @brief Places the MCU into stop mode. In this mode, all clocks are stopped but register, RAM, and GPIO states are
 * saved. The MCU will wake up on either an interrupt or an event, depending on the passed parameter.
 *
 * @param source whether to wakeup on an interrupt or an event
 */
void enter_stop_mode(WakeupSource source);

/**
 * @brief Places the MCU into standby mode. In this mode, the entire 1.8 volt domain is depowered. The MCU is
 * effectively reset when woken from standby.
 */
[[noreturn]] void enter_standby_mode();

/**
 * @brief Enables and calibrates the given ADC.
 *
 * @param adc the target ADC peripheral
 * @param channel_count the sequence length of the regular group
 */
void adc_init(ADC_TypeDef *adc, std::uint32_t channel_count);

/**
 * @brief Disables the given ADC to reduce power usage.
 *
 * @param adc the target ADC peripheral
 */
void adc_deinit(ADC_TypeDef *adc);

/**
 * @brief Enables DMA in a circular, memory-increment mode for ADC1. Note that ADC2 doesn't support DMA.
 *
 * @param data the DMA destination buffer
 */
void adc_init_dma(std::span<volatile std::uint16_t> data);

/**
 * @brief Sets the channel to be sequenced at the given index. Refer to the datasheet for sample time meaning.
 *
 * @param adc the target ADC peripheral
 * @param index the 1-based sequence index
 * @param channel the 0-based ADC channel number
 * @param sample_time 3-bit sample time
 */
void adc_sequence_channel(ADC_TypeDef *adc, std::uint32_t index, std::uint32_t channel, std::uint32_t sample_time);

/**
 * @brief Issues a software start to the given ADC.
 *
 * @param adc the target ADC peripheral
 */
void adc_start(ADC_TypeDef *adc);

/**
 * @brief Computes the 32-bit CRC of the given data using the Ethernet polynomial.
 *
 * @param data the data buffer
 */
std::uint32_t crc_compute(std::span<const std::uint8_t> data);

/**
 * @brief Spins for the specified amount of microseconds. This function modifies the SysTick timer state.
 *
 * @param us the time to wait in microseconds
 */
void delay_us(std::size_t us);

/**
 * @brief Resets and enables the given I2C peripheral.
 *
 * @param i2c the target I2C peripheral
 * @param own_address an optional slave address to set
 */
void i2c_init(I2C_TypeDef *i2c, std::optional<std::uint8_t> own_address);

/**
 * @brief Receives bytes from the slave device specified by the given I2C address on the bus connected to the given
 * peripheral.
 *
 * @param i2c the target I2C peripheral
 * @param address the slave address
 * @param data the data buffer to receive bytes into
 * @param timeout the maximum time to wait for each incoming byte in milliseconds
 * @return I2cStatus::Ok if the receive was successful
 * @return I2cStatus::Timeout if we failed to assert a START condition on the bus
 * @return I2cStatus::AcknowledgeFailure if no slave acknowledged the address in time
 * @return I2cStatus::Timeout if the slave took too long to transmit a byte, or sent less bytes than expected
 */
I2cStatus i2c_master_read(I2C_TypeDef *i2c, std::uint8_t address, std::span<std::uint8_t> data, std::uint32_t timeout);

/**
 * @brief Transmits the given data buffer to the slave device specified by the given I2C address on the bus connected to
 * the given peripheral.
 *
 * @param i2c the target I2C peripheral
 * @param address the slave address
 * @param data the data buffer to transmit
 * @return I2cStatus::Ok if the transmit was successful
 * @return I2cStatus::Timeout if we failed to assert a START condition on the bus
 * @return I2cStatus::AcknowledgeFailure if no slave acknowledged the address in time
 * @return I2cStatus::AcknowledgeFailure if the slave failed to acknowledge a transmitted byte in time
 * @return I2cStatus::Timeout if the transmit buffer remained full for too long
 */
I2cStatus i2c_master_write(I2C_TypeDef *i2c, std::uint8_t address, std::span<const std::uint8_t> data);

/**
 * @brief Synchronously waits to for the bus master to select our own address and accepts
 *
 * This function is useful if a master transfer request is expected from a source other than the regular address matched
 * interrupt, e.g. an EXTI on another GPIO pin.
 *
 * @param i2c the target I2C peripheral
 * @param timeout the maximum time to wait for address match in milliseconds
 * @return I2cStatus::OkRead if the master wants to transmit data to us
 * @return I2cStatus::OkWrite if the master wants to receive data from us
 * @return I2cStatus::Timeout if our address wasn't matched in time
 */
I2cStatus i2c_slave_accept(I2C_TypeDef *i2c, std::uint32_t timeout);

/**
 * @brief Receives bytes from the bus master into the given data buffer.
 *
 * @param i2c the target I2C peripheral
 * @param data the data buffer to receive into
 * @param timeout the maximum time to wait for each incoming byte in milliseconds
 * @return I2cStatus::Ok if the receive was successful
 * @return I2cStatus::Timeout if the master took too long to transmit a byte, or sent less bytes than expected
 */
I2cStatus i2c_slave_read(I2C_TypeDef *i2c, std::span<std::uint8_t> data, std::uint32_t timeout);

/**
 * @brief Replies to the bus master with the given data buffer.
 *
 * @param i2c the target I2C peripheral
 * @param data the data to reply to the master with
 * @param timeout the maximum time to wait for the master to acknowledge the transmitted bytes in milliseconds
 * @return I2cStatus::Ok if the transmit was successful
 * @return I2cStatus::Timeout if the master didn't acknowledge a transmitted byte in time
 * @return I2cStatus::Timeout if the transmit buffer remained full for too long
 */
I2cStatus i2c_slave_write(I2C_TypeDef *i2c, std::span<const std::uint8_t> data, std::uint32_t timeout);

/**
 * @brief Asserts a STOP condition on the I2C bus connected to the given peripheral.
 *
 * @param i2c the target I2C peripheral
 */
void i2c_stop(I2C_TypeDef *i2c);

/**
 * @brief Waits until the I2C bus connected to the given peripheral is idle, up to the specified timeout.
 *
 * @param i2c the target I2C peripheral
 * @param timeout the maximum time to wait in milliseconds
 */
I2cStatus i2c_wait_idle(I2C_TypeDef *i2c, std::uint32_t timeout);

/**
 * @brief Resets and enables the given SPI peripheral in master mode with software slave management.
 *
 * @param spi the target SPI peripheral
 * @param baud_rate the baud rate divisor; must be one of the `SPI_CR1_BR_x` values
 */
void spi_init_master(SPI_TypeDef *spi, std::uint32_t baud_rate);

/**
 * @brief Performs a bidirectional transfer on the SPI bus connected to the given SPI peripheral. The bytes in `data`
 * will be written out on the bus and the buffer will be overwritten with the same number of received bytes from the
 * slave.
 *
 * @param spi the target SPI peripheral
 * @param data the bidirectional buffer to transmit from and receive the slave's response into
 * @param timeout the maximum time to wait for transmit and receive readiness in milliseconds
 */
bool spi_transfer(SPI_TypeDef *spi, std::span<std::uint8_t> data, std::uint32_t timeout);

/**
 * @brief Writes a single character to the SWO SWD output.
 *
 * @param ch the character to write
 */
void swd_putc(char ch);

/**
 * @brief Writes the given format string to the SWO SWD output, replacing any format specifiers with any variable
 * arguments passed.
 *
 * @param format the format string to write
 * @param ... additional arguments to format into the written string
 * @return the number of characters written
 */
__attribute__((format(printf, 1, 2))) int swd_printf(const char *format, ...);

/**
 * @brief Waits until the register ANDed with the given mask is equal to the desired value, or the timeout expires.
 *
 * @param reg the MMIO register
 * @param mask a mask to AND the register's value with
 * @param desired the desired value
 * @param timeout a timeout in milliseconds
 * @return true if the register now equals the desired value; false otherwise
 */
// TODO: Make this nodiscard.
bool wait_equal(const volatile std::uint32_t &reg, std::uint32_t mask, std::uint32_t desired,
                std::uint32_t timeout = UINT32_MAX);

} // namespace hal

bool hal_low_power();
