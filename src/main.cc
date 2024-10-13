#include <stm32f103xb.h>

int main() {
    uint32_t cfgr = RCC->CFGR;
    cfgr &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    cfgr |= RCC_CFGR_HPRE_DIV1;
    cfgr |= RCC_CFGR_PPRE1_DIV16;
    cfgr |= RCC_CFGR_PPRE2_DIV16;
    RCC->CFGR = cfgr;

    // Enable GPIOA clock.
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    // Set PA1 to a push-pull low-speed output.
    GPIOA->CRL &= ~0xf0;
    GPIOA->CRL |= 0x10;

    bool on = false;
    while (true) {
        on = !on;
        if (on) {
            GPIOA->ODR |= 0b10u;
        } else {
            GPIOA->ODR &= ~0b10u;
        }
        
        
        for (uint32_t i = 0; i < 200000; i++) {
            asm volatile("" ::: "memory");
        }
    }
}
