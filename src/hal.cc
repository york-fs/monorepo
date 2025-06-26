#include <hal.hh>

#include <miniprintf.h>
#include <stm32f103xb.h>
#include <util.hh>

#include <bit>
#include <cstdarg>
#include <cstdint>
#include <optional>
#include <span>

[[gnu::weak]] bool hal_low_power() {
    return false;
}

namespace hal {
namespace {

void set_gpio(GPIO_TypeDef *port, std::uint32_t pin, std::uint32_t cnf, std::uint32_t mode) {
    const auto shift = (pin % 8) * 4;
    auto &reg = pin > 7 ? port->CRH : port->CRL;
    reg &= ~(0xf << shift);
    reg |= cnf << (shift + 2);
    reg |= mode << shift;
}

GPIO_TypeDef *gpio_port(GpioPort port) {
    return std::array{
        GPIOA, GPIOB, GPIOC, GPIOD, GPIOE,
    }[static_cast<std::uint32_t>(port)];
}

template <typename Predicate>
bool wait_until(std::uint32_t timeout, Predicate &&predicate) {
    // Start SysTick with a 1 ms period.
    SysTick->LOAD = (hal_low_power() ? 8000 : 56000) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    while (timeout > 0 && !predicate()) {
        // Reading from SysTick->CTRL clears the underflow flag.
        if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0) {
            timeout--;
        }
    }
    SysTick->CTRL = 0;
    return predicate();
}

} // namespace

Gpio::Gpio(GpioPort port, std::uint8_t pin) : m_port(gpio_port(port)), m_pin(pin) {}

void Gpio::configure(GpioInputMode mode) const {
    auto cnf_bits = static_cast<std::uint32_t>(mode);
    if (mode == GpioInputMode::PullDown || mode == GpioInputMode::PullUp) {
        cnf_bits = 0b10u;
    }
    set_gpio(m_port, m_pin, cnf_bits, 0b00u);
    if (mode == GpioInputMode::PullUp) {
        hal::gpio_set(*this);
    } else if (mode == GpioInputMode::PullDown) {
        hal::gpio_reset(*this);
    }
}

void Gpio::configure(GpioOutputMode mode, GpioOutputSpeed speed) const {
    set_gpio(m_port, m_pin, static_cast<std::uint32_t>(mode), static_cast<std::uint32_t>(speed));
    hal::gpio_reset(*this);
}

void enable_irq(IRQn_Type irq, std::uint32_t priority) {
    NVIC_SetPriority(irq, priority);
    NVIC_EnableIRQ(irq);
}

void disable_irq(IRQn_Type irq) {
    NVIC_DisableIRQ(irq);
}

// Workaround for STM32F103 erratum affecting stop debug mode.
[[gnu::noinline]] static void wfe() {
    __WFE();
    __NOP();
}

void enter_stop_mode() {
    // Clear the PDDS bit to ensure stop mode, not standby mode, is selected.
    PWR->CR &= ~PWR_CR_PDDS;

    // Set the SLEEPDEEP bit to set stop mode rather than sleep mode.
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    // Set event and invoke WFE twice to clear any stale events.
    __SEV();
    wfe();
    wfe();

    // Clear the SLEEPDEEP bit.
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
}

void adc_init(ADC_TypeDef *adc, std::uint32_t channel_count) {
    // Enable clock for ADC.
    RCC->APB2ENR |= (adc == ADC1 ? RCC_APB2ENR_ADC1EN : RCC_APB2ENR_ADC2EN);

    // Wait for ADC to settle.
    adc->CR2 |= ADC_CR2_ADON;
    hal::delay_us(100);

    // Perform calibration.
    adc->CR2 |= ADC_CR2_RSTCAL;
    hal::wait_equal(adc->CR2, ADC_CR2_RSTCAL, 0u);
    adc->CR2 |= ADC_CR2_CAL;
    hal::wait_equal(adc->CR2, ADC_CR2_CAL, 0u);

    // Default to external trigger via software start.
    adc->CR2 |= 0b111u << ADC_CR2_EXTSEL_Pos;

    // If we have more than one channel, enable scan mode, since we probably always want it.
    adc->SQR1 |= (channel_count - 1) << ADC_SQR1_L_Pos;
    if (channel_count > 1) {
        adc->CR1 |= ADC_CR1_SCAN;
    }
}

void adc_init_dma(std::span<std::uint16_t> data) {
    // Enable DMA peripheral clock.
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    // Enable DMA mode on ADC1.
    ADC1->CR2 |= ADC_CR2_DMA;

    // Configure DMA channel 1.
    DMA1_Channel1->CPAR = std::bit_cast<std::uint32_t>(&ADC1->DR);
    DMA1_Channel1->CMAR = std::bit_cast<std::uint32_t>(data.data());
    DMA1_Channel1->CNDTR = data.size();
    DMA1_Channel1->CCR = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_EN;
}

void adc_sequence_channel(ADC_TypeDef *adc, std::uint32_t index, std::uint32_t channel, std::uint32_t sample_time) {
    // Enable temperature/VREF channel.
    if (adc == ADC1 && (channel == 16 || channel == 17)) {
        adc->CR2 |= ADC_CR2_TSVREFE;
    }

    // Configure sample time.
    if (channel >= 10) {
        adc->SMPR1 |= sample_time << ((channel - 10) * 3);
    } else {
        adc->SMPR2 |= sample_time << (channel * 3);
    }

    // Map sequence index to channel.
    if (index >= 13) {
        adc->SQR1 |= channel << ((index - 13) * 5);
    } else if (index >= 7) {
        adc->SQR2 |= channel << ((index - 7) * 5);
    } else {
        adc->SQR3 |= channel << ((index - 1) * 5);
    }
}

void adc_start(ADC_TypeDef *adc) {
    // Clear EOC flag.
    adc->SR |= ADC_SR_EOC;

    // Issue software start.
    adc->CR2 |= ADC_CR2_SWSTART | ADC_CR2_EXTTRIG;
}

void delay_us(std::size_t us) {
    // Use a 2 us tick rate on 8 MHz to mitigate the slowness of the loop.
    SysTick->LOAD = (hal_low_power() ? 16 : 56) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    if (hal_low_power()) {
        us /= 2;
    }
    for (std::size_t i = 0; i < us; i++) {
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0) {
        }
    }
    SysTick->CTRL = 0;
}

void i2c_init(I2C_TypeDef *i2c, std::optional<std::uint8_t> own_address) {
    // Enable peripheral clock.
    RCC->APB1ENR |= (i2c == I2C1 ? RCC_APB1ENR_I2C1EN : RCC_APB1ENR_I2C2EN);

    // Configure peripheral clock frequency.
    i2c->CR2 = hal_low_power() ? 8u : 28u;

    // Configure for 100 kHz.
    // TODO: Calculate this properly and allow different speeds.
    if (hal_low_power()) {
        i2c->CCR = 40u;
        i2c->TRISE = 9u;
    } else {
        i2c->CCR = 140u;
        i2c->TRISE = 29u;
    }

    // Set own address if supplied.
    if (own_address) {
        i2c->OAR1 = *own_address << 1;
    }

    // Enable peripheral.
    i2c->CR1 = I2C_CR1_PE;
}

static bool i2c_wait_af(I2C_TypeDef *i2c, std::uint32_t flag) {
    return hal::wait_until(1, [&] {
        return (i2c->SR1 & (I2C_SR1_AF | flag)) != 0u;
    });
}

static I2cStatus i2c_begin(I2C_TypeDef *i2c, std::uint8_t address) {
    // Clear status flags.
    i2c->SR1 = 0u;

    // Send start.
    i2c->CR1 |= I2C_CR1_START;
    if (!hal::wait_equal(i2c->SR1, I2C_SR1_SB, I2C_SR1_SB, 1)) {
        return I2cStatus::Timeout;
    }

    // Send address and wait for acknowledge or acknowledge failure.
    i2c->DR = address;
    const bool timed_out = !i2c_wait_af(i2c, I2C_SR1_ADDR);
    if ((i2c->SR1 & I2C_SR1_AF) != 0u) {
        return I2cStatus::AcknowledgeFailure;
    }
    if (timed_out || (i2c->SR1 & I2C_SR1_ADDR) == 0u) {
        return I2cStatus::Timeout;
    }

    // Clear address sent flag.
    i2c->SR2;
    return I2cStatus::Ok;
}

I2cStatus i2c_master_read(I2C_TypeDef *i2c, std::uint8_t address, std::span<std::uint8_t> data, std::uint32_t timeout) {
    // Enable acknowledge if needed before receiving the first data.
    i2c->CR1 &= ~(I2C_CR1_POS | I2C_CR1_ACK);
    if (data.size() >= 2) {
        i2c->CR1 |= I2C_CR1_ACK;
        if (data.size() == 2) {
            // Handle 2 byte edge case.
            i2c->CR1 |= I2C_CR1_POS;
        }
    }

    // Begin transaction with the read bit set.
    if (auto status = i2c_begin(i2c, (address << 1) | 1u); status != I2cStatus::Ok) {
        return status;
    }

    // Handle 2 byte edge case.
    if (data.size() == 2) {
        i2c->CR1 &= ~I2C_CR1_ACK;
    }

    // Read bytes.
    std::size_t index = 0;
    while (index < data.size()) {
        const auto bytes_remaining = data.size() - index;
        if (bytes_remaining == 3) {
            // Last 3 bytes - wait for the current byte to be flushed from the shift register and disable the ACK flag
            // ready to NACK the last byte.
            if (!hal::wait_equal(i2c->SR1, I2C_SR1_BTF, I2C_SR1_BTF, timeout)) {
                return I2cStatus::Timeout;
            }
            i2c->CR1 &= ~I2C_CR1_ACK;
        }
        if (!hal::wait_equal(i2c->SR1, I2C_SR1_RXNE, I2C_SR1_RXNE, timeout)) {
            return I2cStatus::Timeout;
        }
        data[index++] = i2c->DR;
    }

    // Wait for the transfer to fully finish.
    if (!hal::wait_equal(i2c->SR1, I2C_SR1_BTF, I2C_SR1_BTF, timeout)) {
        return I2cStatus::Timeout;
    }
    return I2cStatus::Ok;
}

I2cStatus i2c_master_write(I2C_TypeDef *i2c, std::uint8_t address, std::span<const std::uint8_t> data) {
    // Begin transaction with the read bit unset.
    i2c->CR1 &= ~(I2C_CR1_POS | I2C_CR1_ACK);
    if (auto status = i2c_begin(i2c, address << 1); status != I2cStatus::Ok) {
        return status;
    }

    // Send bytes.
    for (std::uint8_t byte : data) {
        if (!i2c_wait_af(i2c, I2C_SR1_TXE)) {
            return I2cStatus::Timeout;
        }
        if ((i2c->SR1 & I2C_SR1_AF) != 0u) {
            i2c->CR1 |= I2C_CR1_STOP;
            return I2cStatus::AcknowledgeFailure;
        }
        i2c->DR = byte;
    }

    // Wait for the transfer to fully finish.
    if (!i2c_wait_af(i2c, I2C_SR1_BTF)) {
        return I2cStatus::Timeout;
    }
    if ((i2c->SR1 & I2C_SR1_AF) != 0u) {
        i2c->CR1 |= I2C_CR1_STOP;
        return I2cStatus::AcknowledgeFailure;
    }
    return I2cStatus::Ok;
}

I2cStatus i2c_slave_accept(I2C_TypeDef *i2c, std::uint32_t timeout) {
    // Enable address acknowledge.
    i2c->CR1 |= I2C_CR1_ACK;

    // Wait for address match.
    if (!hal::wait_equal(i2c->SR1, I2C_SR1_ADDR, I2C_SR1_ADDR, timeout)) {
        return I2cStatus::Timeout;
    }
    return ((i2c->SR2 & I2C_SR2_TRA) == 0u) ? I2cStatus::OkRead : I2cStatus::OkWrite;
}

I2cStatus i2c_slave_read(I2C_TypeDef *i2c, std::span<std::uint8_t> data, std::uint32_t timeout) {
    // Enable acknowledgement of bytes.
    i2c->CR1 |= I2C_CR1_ACK;

    // Read bytes.
    for (auto &byte : data) {
        if (!hal::wait_equal(i2c->SR1, I2C_SR1_RXNE, I2C_SR1_RXNE, timeout)) {
            i2c->CR1 &= ~I2C_CR1_ACK;
            return I2cStatus::Timeout;
        }
        byte = i2c->DR;
    }
    return I2cStatus::Ok;
}

I2cStatus i2c_slave_write(I2C_TypeDef *i2c, std::span<const std::uint8_t> data, std::uint32_t timeout) {
    for (auto byte : data) {
        if (!hal::wait_equal(i2c->SR1, I2C_SR1_TXE, I2C_SR1_TXE, timeout)) {
            return I2cStatus::Timeout;
        }
        i2c->DR = byte;
    }

    if (!hal::wait_equal(i2c->SR1, I2C_SR1_AF, I2C_SR1_AF, timeout)) {
        return I2cStatus::Timeout;
    }
    i2c->SR1 &= ~I2C_SR1_AF;
    return I2cStatus::Ok;
}

void i2c_stop(I2C_TypeDef *i2c) {
    i2c->CR1 |= I2C_CR1_STOP;
}

I2cStatus i2c_wait_idle(I2C_TypeDef *i2c) {
    // Allow 25 ms for bus idle.
    return hal::wait_equal(i2c->SR2, I2C_SR2_BUSY, 0, 25) ? I2cStatus::Ok : I2cStatus::Timeout;
}

void spi_init_master(SPI_TypeDef *spi, std::uint32_t baud_rate) {
    // Enable peripheral clock.
    if (spi == SPI1) {
        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    } else {
        RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    }
    spi->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE | baud_rate | SPI_CR1_MSTR;
}

bool spi_transfer(SPI_TypeDef *spi, const Gpio &chip_select, std::span<std::uint8_t> data, std::uint32_t timeout) {
    // Pull CS low and create a scope guard to pull it high again on return.
    util::ScopeGuard cs_guard([&chip_select] {
        hal::gpio_set(chip_select);
    });
    hal::gpio_reset(chip_select);

    // Transmit and receive each byte.
    for (auto &byte : data) {
        // Transmit byte.
        if (!hal::wait_equal(spi->SR, SPI_SR_TXE, SPI_SR_TXE, timeout)) {
            return false;
        }
        spi->DR = byte;

        // Receive byte.
        if (!hal::wait_equal(spi->SR, SPI_SR_RXNE, SPI_SR_RXNE, timeout)) {
            return false;
        }
        byte = spi->DR;
    }

    // Wait for busy to clear before resetting CS to high.
    return hal::wait_equal(spi->SR, SPI_SR_BSY, 0u, timeout);
}

void swd_putc(char ch) {
    ITM_SendChar(ch);
}

int swd_printf(const char *format, ...) {
    va_list args;
    va_start(args, format);
    int rc = mini_vprintf_cooked(swd_putc, format, args);
    va_end(args);
    return rc;
}

bool wait_equal(const volatile std::uint32_t &reg, std::uint32_t mask, std::uint32_t desired, std::uint32_t timeout) {
    return wait_until(timeout, [&] {
        return (reg & mask) == desired;
    });
}

} // namespace hal

extern void app_main();

int main() {
    // Enable 56 MHz system clock via an 8 MHz external crystal if low power mode is not desired.
    if (!hal_low_power()) {
        // Increase flash latency for use with a 56 MHz AHB clock.
        FLASH->ACR |= FLASH_ACR_LATENCY_2;

        // Enable HSE (8 MHz crystal) and wait for readiness.
        RCC->CR |= RCC_CR_HSEON;
        hal::wait_equal(RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY);

        // Configure PLL to HSE * 7 = 56 MHz.
        uint32_t rcc_cfgr = RCC->CFGR;
        rcc_cfgr &= ~RCC_CFGR_PLLMULL;
        rcc_cfgr |= RCC_CFGR_PLLMULL7;
        rcc_cfgr |= RCC_CFGR_PLLSRC;
        RCC->CFGR = rcc_cfgr;

        // Enable PLL and wait for readiness.
        RCC->CR |= RCC_CR_PLLON;
        hal::wait_equal(RCC->CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY);

        // Switch system clock to PLL. HSI is default, so no need to mask.
        RCC->CFGR |= RCC_CFGR_SW_PLL;
        hal::wait_equal(RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL);

        // Done with the HSI, disable it.
        RCC->CR &= ~RCC_CR_HSION;
        hal::wait_equal(RCC->CR, RCC_CR_HSIRDY, 0u);

        // Set a 2x divider on APB1 clock as to not exceed 36 MHz limit.
        RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

        // Set a 4x divider on the ADC clock to achieve the maximum 14 MHz.
        RCC->CFGR |= RCC_CFGR_ADCPRE_DIV4;
    }

    // Default to setting the internal LDO to a low-power mode in stop mode. This incurs a small startup time penalty.
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_LPDS;

    // Disable JTAG interface.
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

    // Enable clocks for all GPIO ports. For cases where we care about power usage, stop and standby mode will disable
    // them anyway. Otherwise, where we don't care about power usage, this simplifies things.
    RCC->APB2ENR |=
        RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPDEN | RCC_APB2ENR_IOPEEN;

    // Default all pins to pull-down.
    for (auto *gpio : {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE}) {
        gpio->CRL = 0x88888888;
        gpio->CRH = 0x88888888;
    }

    // Jump to user code.
    app_main();
}
