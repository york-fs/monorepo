#include <hal.hh>

#include <miniprintf.h>
#include <stm32f103xb.h>

#include <bit>
#include <cstdarg>
#include <cstdint>
#include <span>

namespace hal {

static volatile uint32_t s_ticks = 0;

extern "C" void SysTick_Handler() {
    s_ticks = s_ticks + 1;
}

static void set_gpio(GPIO_TypeDef *port, std::uint32_t pin, std::uint32_t cnf, std::uint32_t mode) {
    const auto shift = (pin % 8) * 4;
    auto &reg = pin > 7 ? port->CRH : port->CRL;
    reg &= ~(0xf << shift);
    reg |= cnf << (shift + 2);
    reg |= mode << shift;
}

void configure_gpio(GPIO_TypeDef *port, std::uint32_t pin, GpioInputMode mode) {
    set_gpio(port, pin, static_cast<std::uint32_t>(mode), 0b00u);
}

void configure_gpio(GPIO_TypeDef *port, std::uint32_t pin, GpioOutputMode mode, GpioOutputSpeed speed) {
    set_gpio(port, pin, static_cast<std::uint32_t>(mode), static_cast<std::uint32_t>(speed));
}

void enable_irq(IRQn_Type irq, std::uint32_t priority) {
    NVIC_SetPriority(irq, priority);
    NVIC_EnableIRQ(irq);
}

void disable_irq(IRQn_Type irq) {
    NVIC_DisableIRQ(irq);
}

void init_sys_tick() {
    // Initialise SysTick at 1 ms.
    SysTick_Config(56000);
}

void delay_ms(std::uint32_t ms) {
    hal::wait_equal(s_ticks, 0xffffffffu, s_ticks + ms);
}

void adc_init(ADC_TypeDef *adc, std::uint32_t channel_count) {
    // Enable clock for ADC.
    RCC->APB2ENR |= (adc == ADC1 ? RCC_APB2ENR_ADC1EN : RCC_APB2ENR_ADC2EN);

    // Wait for ADC to settle.
    adc->CR2 |= ADC_CR2_ADON;
    for (std::size_t i = 0; i < 56000; i++) {
        asm volatile("" ::: "memory");
    }

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

void adc_init_dma(ADC_TypeDef *adc, DMA_Channel_TypeDef *dma_channel, std::span<std::uint16_t> data) {
    // Enable DMA peripheral clock.
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    // Enable DMA mode on ADC.
    adc->CR2 |= ADC_CR2_DMA;

    // Configure DMA channel.
    dma_channel->CPAR = std::bit_cast<std::uint32_t>(&adc->DR);
    dma_channel->CMAR = std::bit_cast<std::uint32_t>(data.data());
    dma_channel->CNDTR = data.size();
    dma_channel->CCR = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_EN;
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

} // namespace hal

extern void app_main();

int main() {
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

    // Jump to user code.
    app_main();
}
