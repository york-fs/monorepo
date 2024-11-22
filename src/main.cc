#include <stm32f103xb.h>

#include "hal.hh"

int main() {
    // Enable HSE (8 MHz crystal) and wait for readiness.
    RCC->CR |= RCC_CR_HSEON;
    hal::wait_equal(RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY);

    // Configure PLL to HSE*4 = 32 MHz.
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

    // Enable CAN, GPIOA, GPIOB, and AFIO clocks.
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;

    // Remap CAN1 to PB8 and PB9.
    AFIO->MAPR |= AFIO_MAPR_CAN_REMAP_REMAP2;

    // Configure PB8 as CAN_RX (floating input).
    GPIOB->CRH &= ~0xf;
    GPIOB->CRH |= 0x4;

    // Configure PB9 as CAN_TX (push-pull alternate function 50 MHz).
    GPIOB->CRH &= ~0xf0;
    GPIOB->CRH |= 0xb0;

    // Request CAN initialisation and wait for completeness.
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

#if 1
    // TIM2 channel 2.
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Configure PA1 to be a push-pull alternate function.
    GPIOA->CRL &= ~0xf0;
    GPIOA->CRL |= 0x90;

    // Configure CAN receive filter.
    CAN1->FMR |= CAN_FMR_FINIT;

    // Set 32-bit scale on filter 0.
    CAN1->FS1R |= CAN_FS1R_FSC0;

    // 32-bit mask mode so the filter is checking (frame & FR2) == FR1.
    CAN1->sFilterRegister[0].FR1 = 0xffffffff;
    CAN1->sFilterRegister[0].FR2 = 0;

    // Enable filter 0.
    CAN1->FA1R |= CAN_FA1R_FACT0;

    // Leave init mode.
    CAN1->FMR &= ~CAN_FMR_FINIT;

    // Configure PWM output.
    TIM2->PSC = 599;
    TIM2->ARR = 999;
    TIM2->CCMR1 |= (6u << TIM_CCMR1_OC2M_Pos);
    TIM2->CCMR1 |= TIM_CCMR1_OC2PE;
    TIM2->CCER |= TIM_CCER_CC2E;
    TIM2->CR1 |= TIM_CR1_CEN;

    while (true) {
        if ((CAN1->RF0R & CAN_RF0R_FMP0) == 0u) {
            continue;
        }

        // Update duty.
        TIM2->CCR2 = CAN1->sFIFOMailBox[0].RDLR;

        // Release FIFO.
        CAN1->RF0R |= CAN_RF0R_RFOM0;
    }
#else
    // Configure PA1 to be a push-pull low-speed output.
    GPIOA->CRL &= ~0xf0;
    GPIOA->CRL |= 0x10;

    uint32_t duty = 0;
    while (true) {
        // Toggle LED.
        if ((GPIOA->ODR & 0b10u) != 0u) {
            GPIOA->ODR &= ~0b10u;
        } else {
            GPIOA->ODR |= 0b10u;
        }

        // Wait TX ready on mailbox 0.
        hal::wait_equal(CAN1->TSR, CAN_TSR_TME0, CAN_TSR_TME0);

        duty += 100;
        duty %= 1000;

        // Set mailbox identifier register and data length.
        CAN1->sTxMailBox[0].TIR = 0x6a5ul << CAN_TI0R_STID_Pos;
        CAN1->sTxMailBox[0].TDTR = 8;
        CAN1->sTxMailBox[0].TDHR = 0;
        CAN1->sTxMailBox[0].TDLR = duty;

        // Request transmission.
        CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;

        for (uint32_t i = 0; i < 2000000; i++) {
            asm volatile("" ::: "memory");
        }
    }
#endif
}
