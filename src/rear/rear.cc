#include <can.hh>
#include <config.hh>
#include <freertos.hh>
#include <hal.hh>
#include <rear/can_messages.hh>
#include <rear/fuse.hh>
#include <stm32f103xb.h>

#include <cstdint>
#include <string_view>

using namespace rear;

namespace {

/**
 * @brief Whether to enable the SWD debug logging task.
 */
constexpr bool k_enable_debug_logs = false;

/**
 * @brief Hard-coded value of the 3V3 rail powering the STM's ADC in 1 mV resolution.
 */
constexpr std::uint32_t k_mcu_vref = 3300;

enum class ExpanderRegister : std::uint8_t {
    InputPort0 = 0x00,
    InputPort1 = 0x01,
    OutputPort0 = 0x02,
    OutputPort1 = 0x03,
    PolarityPort0 = 0x04,
    PolarityPort1 = 0x05,
    ConfigurationPort0 = 0x06,
    ConfigurationPort1 = 0x07,
};

hal::Gpio s_radio_tx(hal::GpioPort::A, 9);
hal::Gpio s_radio_rx(hal::GpioPort::A, 10);
hal::Gpio s_radio_cts(hal::GpioPort::A, 11);
hal::Gpio s_radio_rts(hal::GpioPort::A, 12);

// Expander I2C.
hal::Gpio s_scl(hal::GpioPort::B, 6);
hal::Gpio s_sda(hal::GpioPort::B, 7);

hal::Gpio s_brake_switch(hal::GpioPort::B, 10);

freertos::Task<128> s_adc_task;
freertos::Task<128> s_expander_task;
freertos::Task<128> s_radio_task;
freertos::Task<128> s_swd_task;

// TODO: Use async I2C.
bool set_expander_register(ExpanderRegister reg, std::uint8_t value) {
    std::array data{
        static_cast<std::uint8_t>(reg),
        value,
    };
    if (const auto status = hal::i2c_wait_idle(I2C1, 5); status != hal::I2cStatus::Ok) {
        return false;
    }
    if (const auto status = hal::i2c_master_write(I2C1, 0x20, data); status != hal::I2cStatus::Ok) {
        return false;
    }
    hal::i2c_stop(I2C1);
    return true;
}

void adc_task(void *) {
    // Initialise CAN on port B.
    can::init(can::Port::B, config::k_can_speed, 1);

    // Enable CAN IRQs.
    hal::irq_enable(CAN1_TX_IRQn, 6);
    hal::irq_enable(CAN1_SCE_IRQn, 5);

    // Configure analog inputs.
    for (std::uint32_t i = 0; i < 8; i++) {
        hal::Gpio(hal::GpioPort::A, i).configure(hal::GpioInputMode::Analog);
    }
    for (std::uint32_t i = 0; i < 2; i++) {
        hal::Gpio(hal::GpioPort::B, i).configure(hal::GpioInputMode::Analog);
    }

    hal::adc_init(ADC1, 10);
    for (std::uint32_t i = 0; i < 10; i++) {
        hal::adc_sequence_channel(ADC1, i, i, 0b010u);
    }

    std::array<std::uint16_t, 10> adc_buffer{};
    hal::adc_init_dma(adc_buffer);

    while (true) {
        for (std::uint32_t i = 0; i < adc_buffer.size(); i++) {
            const auto voltage = (((k_mcu_vref * adc_buffer[i]) >> 12) * 57) / 10;
            hal::swd_printf("%u: %u\n", i, voltage);
        }

        rear::FlashMessage flash_message{};
        can::transmit(config::k_rear_can_id, flash_message);

        hal::adc_start(ADC1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void expander_task(void *) {
    // Configure brake switch.
    s_brake_switch.configure(hal::GpioInputMode::PullDown);

    // Configure I2C for expander.
    s_scl.configure(hal::GpioOutputMode::AlternateOpenDrain, hal::GpioOutputSpeed::Max2);
    s_sda.configure(hal::GpioOutputMode::AlternateOpenDrain, hal::GpioOutputSpeed::Max2);
    hal::i2c_init(I2C1, std::nullopt);

    TickType_t last_schedule_time = xTaskGetTickCount();
    while (true) {
        // Port 0 has all shutdown inputs.
        set_expander_register(ExpanderRegister::ConfigurationPort0, 0xff);
        set_expander_register(ExpanderRegister::ConfigurationPort1, 0xee);
        set_expander_register(ExpanderRegister::OutputPort1, s_brake_switch.read() ? 0xff : 0xef);

        xTaskDelayUntil(&last_schedule_time, pdMS_TO_TICKS(50));
    }
}

void radio_task(void *) {
    // Delay to allow the radio to leave its bootloader.
    vTaskDelay(pdMS_TO_TICKS(50));

    // Configure GPIOs.
    s_radio_tx.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max10);
    s_radio_rx.configure(hal::GpioInputMode::Floating);
    s_radio_cts.configure(hal::GpioInputMode::PullUp);
    s_radio_rts.configure(hal::GpioInputMode::PullUp);

    // Enable peripheral clock.
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // Program 57600 baud rate.
    USART1->BRR = 56000000 / 57600;

    // Enable transmit and receive with interrupts.
    USART1->CR1 = USART_CR1_UE | USART_CR1_TXEIE | USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE;

    // Enable the interrupt.
    hal::irq_enable(USART1_IRQn, 7);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void swd_task(void *) {
    TickType_t last_schedule_time = xTaskGetTickCount();
    while (true) {
        xTaskDelayUntil(&last_schedule_time, pdMS_TO_TICKS(1000));

        hal::swd_printf("--------------------------------\n");
        hal::swd_printf("Uptime: %u\n", freertos::uptime_ms() / 1000);

        const auto can_stats = can::get_stats();
        hal::swd_printf("CAN status: %s %u/%u %u/%u\n", can::is_online() ? "online" : "offline", can_stats.rx_count,
                        can_stats.lost_rx_count, can_stats.tx_count, can_stats.lost_tx_count);
    }
}

} // namespace

extern "C" void USART1_IRQHandler() {
    const auto sr = USART1->SR;
    if ((sr & USART_SR_TXE) != 0) {
        USART1->DR = 'a';
    }
    if ((sr & USART_SR_RXNE) != 0) {
        hal::swd_printf("received byte: 0x%x\n", USART1->DR);
    }
}

void vApplicationIdleHook() {
    hal::enter_sleep_mode(hal::WakeupSource::Interrupt);
}

void app_main() {
    s_adc_task.init(&adc_task, "adc", 2);
    s_expander_task.init(&expander_task, "expander", 1);
    s_radio_task.init(&radio_task, "radio", 1);
    if constexpr (k_enable_debug_logs) {
        s_swd_task.init(&swd_task, "swd", 0);
    }
    vTaskStartScheduler();
}
