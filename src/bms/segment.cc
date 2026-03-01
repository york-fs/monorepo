#include <hal.hh>
#include <stm32f103xb.h>

#include <FreeRTOS.h>
#include <task.h>

#include <cstdint>

namespace {

/**
 * @brief Whether to enable the SWD debug logging task.
 */
constexpr bool k_enable_debug_logs = true;

/**
 * @brief Sleep timeout in milliseconds.
 */
constexpr std::uint32_t k_sleep_timeout = 2000;

TaskHandle_t s_sleep_task_handle = nullptr;

// TODO
std::array s_address_pins{
    hal::Gpio(hal::GpioPort::A, 8),
    hal::Gpio(hal::GpioPort::A, 9),
    hal::Gpio(hal::GpioPort::A, 10),
    hal::Gpio(hal::GpioPort::A, 11),
};

// Thermistors connected directly to the STM.
std::array s_mcu_thermistor_enable{
    hal::Gpio(hal::GpioPort::B, 9), hal::Gpio(hal::GpioPort::B, 8), hal::Gpio(hal::GpioPort::B, 12),
    hal::Gpio(hal::GpioPort::A, 1), hal::Gpio(hal::GpioPort::A, 2), hal::Gpio(hal::GpioPort::A, 3),
    hal::Gpio(hal::GpioPort::A, 4),
};

// TODO
hal::Gpio s_adc_cs(hal::GpioPort::A, 5);
hal::Gpio s_afe_cs(hal::GpioPort::A, 7);
hal::Gpio s_afe_en(hal::GpioPort::B, 0);
hal::Gpio s_ref_en(hal::GpioPort::B, 1);
hal::Gpio s_led(hal::GpioPort::B, 5);
hal::Gpio s_sck(hal::GpioPort::B, 13);
hal::Gpio s_miso(hal::GpioPort::B, 14);
hal::Gpio s_mosi(hal::GpioPort::B, 15);
hal::Gpio s_scl_1(hal::GpioPort::B, 6);
hal::Gpio s_sda_1(hal::GpioPort::B, 7);
hal::Gpio s_scl_2(hal::GpioPort::B, 10);
hal::Gpio s_sda_2(hal::GpioPort::B, 11);

void sleep_task(void *) {
    // TODO
    const auto i2c_address = 0x40u | ~(GPIOA->IDR >> 8u) & 0xfu;

    hal::swd_printf("Hello world 0x%x\n", i2c_address);
    
    while (true) {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(k_sleep_timeout)) != 0) {
            // Notified to stay awake.
            continue;
        }

        hal::swd_printf("Sleeping\n");

        // Reconfigure SCK and MOSI as regular GPIOs as the peripheral will be deactivated.
        s_sck.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
        s_mosi.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

        // Pull the SPI CS lines high by default (active-low).
        // TODO: Cleanup.
        hal::gpio_set(s_adc_cs, s_afe_cs, s_sck);
        hal::gpio_reset(s_adc_cs);
        hal::gpio_set(s_adc_cs);

        // Turn off LED.
        hal::gpio_reset(s_led);

        // Reconfigure SCL as a regular input for use as an external event and enter stop mode. Also reconfigure SDA to
        // avoid the STM driving it low and upsetting the isolator.
        for (const auto &pin : {s_scl_1, s_sda_1, s_scl_2, s_sda_2}) {
            pin.configure(hal::GpioInputMode::Floating);
        }

        // Enter stop mode and wait for external event on SCL (PB6).
        AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI6_PB;
        EXTI->EMR |= EXTI_EMR_MR6;
        EXTI->FTSR |= EXTI_FTSR_TR6;
        SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
        hal::enter_stop_mode(hal::WakeupSource::Event);
        SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

        hal::swd_printf("Awoken\n");

        // Woken up from stop mode.
        hal::gpio_set(s_led);

        for (const auto &pin : {s_scl_1, s_sda_1, s_scl_2, s_sda_2}) {
            pin.configure(hal::GpioOutputMode::AlternateOpenDrain, hal::GpioOutputSpeed::Max2);
        }

        hal::i2c_init(I2C1, i2c_address);

        I2C1->CR2 |= I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN;
        hal::enable_irq(I2C1_EV_IRQn, 6);

        I2C1->CR1 |= I2C_CR1_ACK;
    }
}

void swd_task(void *) {
    TickType_t last_schedule_time = xTaskGetTickCount();
    while (true) {
        hal::swd_printf("SWD\n");
        vTaskDelayUntil(&last_schedule_time, pdMS_TO_TICKS(1000));
    }
}

} // namespace

extern "C" void I2C1_EV_IRQHandler() {
    BaseType_t higher_priority_task_woken = pdFALSE;
    xTaskNotifyFromISR(s_sleep_task_handle, 1, eIncrement, &higher_priority_task_woken);
    
    const auto sr1 = I2C1->SR1;
    if (sr1 & I2C_SR1_ADDR) {
        I2C1->SR2;
        hal::swd_printf("addr matched!\n");
    }

    if (sr1 & I2C_SR1_RXNE) {
        const auto byte = I2C1->DR;
        hal::swd_printf("received byte: 0x%x\n", byte);
    }

    if (sr1 & I2C_SR1_STOPF) {
        I2C1->CR1 |= 0;
        hal::swd_printf("done!\n");
    }

    portYIELD_FROM_ISR(higher_priority_task_woken);
}

void vApplicationIdleHook() {
    // TODO: Test and disable some clocks.
    // hal::enter_sleep_mode(hal::WakeupSource::Interrupt);
}

bool hal_low_power() {
    // Use the 8 MHz internal clock.
    return true;
}

void app_main() {
    // Configure general GPIOs.
    for (const auto &gpio : s_address_pins) {
        gpio.configure(hal::GpioInputMode::PullUp);
    }
    for (const auto &gpio : s_mcu_thermistor_enable) {
        gpio.configure(hal::GpioInputMode::Floating);
    }
    s_ref_en.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_afe_en.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

    // Configure CS pin and enable a pull-up on MISO to avoid floating when no slave is selected.
    s_adc_cs.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_afe_cs.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_miso.configure(hal::GpioInputMode::PullUp);

    xTaskCreate(&sleep_task, "sleep", 128, nullptr, 4, &s_sleep_task_handle);
    if constexpr (k_enable_debug_logs) {
        xTaskCreate(&swd_task, "swd", 128, nullptr, 1, nullptr);
    }
    vTaskStartScheduler();
}
