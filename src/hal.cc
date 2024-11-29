#include <hal.hh>
#include <miniprintf.h>
#include <stm32f103xb.h>

#include <cstdarg>
#include <cstdint>

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

void delay_ms(std::uint32_t ms) {
    hal::wait_equal(s_ticks, 0xffffffffu, s_ticks + ms);
}

void init_clocks() {
    // Enable HSE (8 MHz crystal) and wait for readiness.
    RCC->CR |= RCC_CR_HSEON;
    hal::wait_equal(RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY);

    // Configure PLL to HSE * 4 = 32 MHz.
    uint32_t rcc_cfgr = RCC->CFGR;
    rcc_cfgr &= ~RCC_CFGR_PLLMULL;
    rcc_cfgr |= RCC_CFGR_PLLMULL4;
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

    // Set a 2x divider on APB1 clock (has CAN peripheral).
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

    // Initialise SysTick at 1 ms.
    SysTick_Config(32000);
}

void init_can() {
    // Ensure peripheral clocks are active.
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;

    // Remap CAN1 to PB8 and PB9.
    AFIO->MAPR |= AFIO_MAPR_CAN_REMAP_REMAP2;

    // Configure PB8 suitable to be CAN_RX and PB9 suitable to be CAN_TX.
    hal::configure_gpio(GPIOB, 8, GpioInputMode::Floating);
    hal::configure_gpio(GPIOB, 9, GpioOutputMode::AlternatePushPull, GpioOutputSpeed::Max50);

    // Request CAN initialisation.
    CAN1->MCR |= CAN_MCR_INRQ;
    hal::wait_equal(CAN1->MSR, CAN_MSR_INAK, CAN_MSR_INAK);

    // Exit sleep mode.
    CAN1->MCR &= ~CAN_MCR_SLEEP;
    hal::wait_equal(CAN1->MSR, CAN_MSR_SLAK, 0u);

    // Set the bit timing register. Peripheral clocked at 16 MHz and 500 kbits/s.
    // The value below sets 13+2+1 (seg1+seg2+sync) time quanta per bit with a prescaler of 2.
    CAN1->BTR = 0x001c0001;

    // Leave initialisation mode.
    CAN1->MCR &= ~CAN_MCR_INRQ;
    hal::wait_equal(CAN1->MSR, CAN_MSR_INAK, 0u);
}

void route_can_filter(const std::uint8_t filter, const std::uint8_t fifo, const std::uint32_t mask,
                      const std::uint32_t value) {
    const std::uint32_t filter_bit = 1u << filter;

    // Ensure filter is disabled.
    CAN1->FA1R &= ~filter_bit;

    // Enable filter init mode.
    CAN1->FMR |= CAN_FMR_FINIT;

    // Set 32-bit scale mask mode.
    CAN1->FM1R &= ~filter_bit;
    CAN1->FS1R |= filter_bit;

    // Set desired FIFO.
    if (fifo != 0u) {
        CAN1->FFA1R |= filter_bit;
    } else {
        CAN1->FFA1R &= ~filter_bit;
    }

    // Set mask and desired value.
    CAN1->sFilterRegister[filter].FR1 = value;
    CAN1->sFilterRegister[filter].FR2 = mask;

    // Enable the filter and leave init mode.
    CAN1->FA1R |= filter_bit;
    CAN1->FMR &= ~CAN_FMR_FINIT;
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
