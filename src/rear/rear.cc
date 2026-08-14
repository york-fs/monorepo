#include <can.hh>
#include <config.hh>
#include <freertos.hh>
#include <front/can_messages.hh>
#include <hal.hh>
#include <i2c.hh>
#include <precharge/can_messages.hh>
#include <rear/can_messages.hh>
#include <rear/fuse.hh>
#include <stm32f103xb.h>
#include <time_tracked.hh>

#include <algorithm>
#include <bit>
#include <cstdint>
#include <limits>
#include <span>

using namespace rear;

namespace {

/**
 * @brief The voltage threshold to consider a fuse working relative to the maximum measured LVS voltage in millivolts.
 */
constexpr std::uint16_t k_fuse_threshold = 1000;

/**
 * @brief The configured baud rate of the radio's UART.
 */
constexpr std::uint32_t k_radio_baud = 115200;

/**
 * @brief Telemetry radio send period in milliseconds.
 */
constexpr std::uint32_t k_radio_period = 100;

/**
 * @brief Hard-coded value of the 3V3 rail powering the STM's ADC in 1 mV resolution.
 */
constexpr std::uint32_t k_mcu_vref = 3300;

/**
 * @brief I2C address of the GPIO expander.
 */
constexpr std::uint8_t k_expander_address = 0x20;

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

struct RadioData {
    FuseBitset fuse_bitset;
    std::uint16_t lvs_min_voltage;
    std::uint16_t lvs_max_voltage;
};

// Latest received statuses from other components.
TimeTracked<front::StatusMessage> s_front_status(250);
TimeTracked<precharge::StatusMessage> s_precharge_status(25);

// Front LVS voltages.
std::array<std::uint16_t, 7> s_front_lvs_voltages{};

// Shutdown samples from expander.
std::uint8_t s_rear_shutdown_samples = 0;
bool s_bms_ok = false;
bool s_imd_ok = false;
bool s_dti_ok = false;

// Queue for consistent radio data.
freertos::Queue<RadioData, 1> s_radio_data;

// I2C state machine for GPIO expander.
i2c::StateMachine s_i2c_sm(i2c::Bus::_1);

hal::Gpio s_radio_tx(hal::GpioPort::A, 9);
hal::Gpio s_radio_rx(hal::GpioPort::A, 10);
hal::Gpio s_radio_cts(hal::GpioPort::A, 11);
hal::Gpio s_radio_rts(hal::GpioPort::A, 12);

hal::Gpio s_scl(hal::GpioPort::B, 6);
hal::Gpio s_sda(hal::GpioPort::B, 7);

hal::Gpio s_dti_ok_sample(hal::GpioPort::B, 2);
hal::Gpio s_brake_switch(hal::GpioPort::B, 10);

freertos::Task<256> s_main_task;
freertos::Task<128> s_expander_task;
freertos::Task<256> s_radio_task;
freertos::Task<128> s_swd_task;

void main_task(void *) {
    // Initialise CAN on port B.
    can::init(can::Port::B, config::k_can_speed, 3);

    // Setup CAN listeners.
    can::listen<front::StatusMessage, [](const front::StatusMessage &front_status) {
        s_front_status.receive(front_status);
    }>(config::k_front_can_id, 0);
    can::listen<precharge::StatusMessage, [](const precharge::StatusMessage &precharge_status) {
        s_precharge_status.receive(precharge_status);
    }>(config::k_precharge_can_id, 1);
    can::listen<front::LvsSampleMessage1, [](const front::LvsSampleMessage1 &lvs_1) {
        s_front_lvs_voltages[0] = lvs_1.rtd_voltage;
        s_front_lvs_voltages[1] = lvs_1.apps_1_voltage;
        s_front_lvs_voltages[2] = lvs_1.apps_2_voltage;
        s_front_lvs_voltages[3] = lvs_1.front_voltage;
    }>(config::k_front_can_id, 2);
    can::listen<front::LvsSampleMessage2, [](const front::LvsSampleMessage2 &lvs_2) {
        s_front_lvs_voltages[4] = lvs_2.dwin_voltage;
        s_front_lvs_voltages[5] = lvs_2.aux_1_voltage;
        s_front_lvs_voltages[6] = lvs_2.aux_2_voltage;
    }>(config::k_front_can_id, 3);

    // Enable CAN IRQs.
    hal::irq_enable(CAN1_RX0_IRQn, 7);
    hal::irq_enable(CAN1_TX_IRQn, 6);
    hal::irq_enable(CAN1_SCE_IRQn, 5);

    hal::adc_init(ADC1, 10);
    for (std::uint32_t i = 0; i < 10; i++) {
        hal::adc_sequence_channel(ADC1, i + 1, i, 0b010u);
    }

    std::array<std::uint16_t, 10> adc_buffer{};
    hal::adc_init_dma(adc_buffer);

    DMA1_Channel1->CCR |= DMA_CCR_TCIE;
    hal::irq_enable(DMA1_Channel1_IRQn, 8);

    // Configure brake switch input.
    s_brake_switch.configure(hal::GpioInputMode::Floating);

    // Main loop which handles fuse and shutdown sampling, precharge heartbeat, and inverter control.
    freertos::PeriodScheduler scheduler;
    while (true) {
        scheduler.delay_until_ms(10);

        // Update all message expiry detections.
        s_front_status.update();
        s_precharge_status.update();

        // Sample all ADC channels.
        hal::adc_start(ADC1);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Create an array of rear and front measured fuse voltages.
        std::array<std::uint16_t, 17> fuse_voltages{};
        if (s_front_status) {
            freertos::in_critical_section([&] {
                auto out_it = std::next(fuse_voltages.begin(), 10);
                std::copy(s_front_lvs_voltages.begin(), s_front_lvs_voltages.end(), out_it);
            });
        }
        std::transform(adc_buffer.begin(), adc_buffer.end(), fuse_voltages.begin(), [](std::uint16_t adc_value) {
            return (k_mcu_vref * adc_value) >> 12;
        });

        // Reverse 5.7x divider on each measured fuse voltage.
        std::transform(fuse_voltages.begin(), fuse_voltages.end(), fuse_voltages.begin(), [](std::uint16_t voltage) {
            return (voltage * 57) / 10;
        });

        // Rate fuse soundness based on the highest voltage we measure.
        std::uint16_t lvs_min_voltage = std::numeric_limits<std::uint16_t>::max();
        const auto lvs_max_voltage = *std::max_element(fuse_voltages.begin(), fuse_voltages.end());
        FuseBitset fuse_bitset;
        for (std::uint16_t fuse = 0; fuse < fuse_voltages.size(); fuse++) {
            if (lvs_max_voltage - fuse_voltages[fuse] <= k_fuse_threshold) {
                // The fuse is deemed working.
                fuse_bitset.set(Fuse(fuse));
                lvs_min_voltage = std::min(lvs_min_voltage, fuse_voltages[fuse]);
            }
        }

        // Update radio data.
        s_radio_data.overwrite(RadioData{
            .fuse_bitset = fuse_bitset,
            .lvs_min_voltage = lvs_min_voltage,
            .lvs_max_voltage = lvs_max_voltage,
        });

        // TODO: Enum with TS and RTD off reason.

        if (!s_front_status || !s_precharge_status) {
            continue;
        }
        if (!s_front_status->ts_activation_desired) {
            continue;
        }
        can::transmit(config::k_precharge_can_id, precharge::HeartbeatMessage{});
    }
}

bool expander_wait() {
    const bool timeout = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10)) == 0;
    const auto state = s_i2c_sm.state();
    if (!timeout && (state == i2c::State::Idle || state == i2c::State::NoAck)) {
        return state == i2c::State::Idle;
    }
    // Reset the I2C periphral.
    s_i2c_sm.init();
    return false;
}

std::optional<std::uint8_t> expander_read(ExpanderRegister reg) {
    std::array data{
        static_cast<std::uint8_t>(reg),
    };
    s_i2c_sm.start_write(k_expander_address, data, false);
    if (!expander_wait()) {
        return std::nullopt;
    }
    s_i2c_sm.start_read(k_expander_address, data, true);
    if (!expander_wait()) {
        return std::nullopt;
    }
    return data[0];
}

bool expander_write(ExpanderRegister reg, std::uint8_t value) {
    std::array data{
        static_cast<std::uint8_t>(reg),
        value,
    };
    s_i2c_sm.start_write(k_expander_address, data, true);
    return expander_wait();
}

void expander_task(void *) {
    // Configure I2C pins for peripheral use.
    s_scl.configure(hal::GpioOutputMode::AlternateOpenDrain, hal::GpioOutputSpeed::Max2);
    s_sda.configure(hal::GpioOutputMode::AlternateOpenDrain, hal::GpioOutputSpeed::Max2);

    // Initialise peripheral and unmask interrupts.
    s_i2c_sm.init();
    hal::irq_enable(I2C1_EV_IRQn, 9);
    hal::irq_enable(I2C1_ER_IRQn, 9);

    // Enable an interrupt on the brake switch line.
    AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI10_PB;
    EXTI->IMR |= EXTI_IMR_MR10;
    EXTI->RTSR |= EXTI_RTSR_RT10;
    EXTI->FTSR |= EXTI_FTSR_FT10;
    hal::irq_enable(EXTI15_10_IRQn, 10);

    // Configure DTI_OK pin which is not on the expander.
    s_dti_ok_sample.configure(hal::GpioInputMode::Floating);

    while (true) {
        // TODO: This could be purely interrupt driven with an interrupt from the GPIO expander.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

        // Set pin config.
        expander_write(ExpanderRegister::PolarityPort0, 0);
        expander_write(ExpanderRegister::PolarityPort1, 0);
        expander_write(ExpanderRegister::ConfigurationPort0, 0xff);
        expander_write(ExpanderRegister::ConfigurationPort1, 0xee);

        // Set brake switch output.
        std::uint8_t port_1_out = 0;
        if (s_brake_switch.read()) {
            port_1_out |= 1u << 4;
        }
        expander_write(ExpanderRegister::OutputPort1, port_1_out);

        // Read shutdown inputs.
        const auto port_0_in = expander_read(ExpanderRegister::InputPort0).value_or(0);
        const auto port_1_in = expander_read(ExpanderRegister::InputPort1).value_or(0);
        freertos::in_critical_section([&] {
            s_rear_shutdown_samples = port_0_in;
            s_bms_ok = (port_1_in & (1u << 2)) != 0;
            s_imd_ok = (port_1_in & (1u << 3)) != 0;
            s_dti_ok = s_dti_ok_sample.read();
        });
    }
}

void radio_task(void *) {
    // Delay to allow the radio to leave its bootloader.
    vTaskDelay(pdMS_TO_TICKS(50));

    // Configure GPIOs.
    s_radio_tx.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max10);
    s_radio_rx.configure(hal::GpioInputMode::Floating);
    s_radio_cts.configure(hal::GpioInputMode::Floating);
    s_radio_rts.configure(hal::GpioInputMode::PullDown);

    // Enable peripheral clocks.
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // Configure DMA with the transfer complete interrupt enabled.
    std::array<std::uint8_t, 256> tx_bytes;
    DMA1_Channel4->CPAR = std::bit_cast<std::uint32_t>(&USART1->DR);
    DMA1_Channel4->CMAR = std::bit_cast<std::uint32_t>(tx_bytes.data());
    DMA1_Channel4->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE;

    // Program the configured baud rate.
    USART1->BRR = 56000000 / k_radio_baud;

    // Enable UART with DMA and transmission enabled.
    USART1->CR3 = USART_CR3_DMAT;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE;

    // Enable the interrupt.
    hal::irq_enable(DMA1_Channel4_IRQn, 7);

    // Keep track of the number of transmission windows we've missed.
    std::uint8_t missed_tx_count = 0;

    freertos::PeriodScheduler scheduler;
    while (true) {
        scheduler.delay_until_ms(k_radio_period);

        // Don't transmit if the radio's UART buffer is near full.
        if (s_radio_cts.read()) {
            missed_tx_count++;
            continue;
        }

        const auto data = *s_radio_data.receive(portMAX_DELAY);

        // Build a telemetry frame with the data offset by one byte to allow for the first COBS code. The size is also
        // limited to 3 bytes fewer to allow for the overall COBS overhead.
        util::Stream stream(std::span<std::uint8_t>(tx_bytes).subspan(1, 253));

        // Append general information.
        stream.write_be(freertos::uptime_ms());
        stream.write_byte(missed_tx_count);

        // Append online status bitset for each component.
        std::uint8_t online_bitset = 0;
        if (s_front_status) {
            online_bitset |= 1u << 0;
        }
        if (s_precharge_status) {
            online_bitset |= 1u << 2;
        }
        stream.write_byte(online_bitset);

        // Append distribution information.
        stream.write_be(data.fuse_bitset.value());
        stream.write_be(data.lvs_min_voltage);
        stream.write_be(data.lvs_max_voltage);

        // Append precharge information.
        stream.write_be(util::to_underlying(s_precharge_status->state));
        stream.write_be(s_precharge_status->error_flags.value());
        stream.write_be(s_precharge_status->precharge_voltage);
        stream.write_be(s_precharge_status->tractive_voltage);
        stream.write_be(s_precharge_status->relay_states.value());

        // Append a checksum.
        stream.write_be(freertos::in_critical_section([&] {
            return hal::crc_compute(stream.bytes());
        }));

        // COBS encode the data array in place.
        std::uint32_t write_index = 1;
        std::uint32_t code_index = 0;
        tx_bytes[code_index] = 1;
        for (std::uint32_t i = 1; i <= stream.head(); i++) {
            const std::uint8_t byte = tx_bytes[i];
            if (byte != 0) {
                tx_bytes[write_index++] = byte;
                tx_bytes[code_index]++;
            } else {
                code_index = write_index++;
                tx_bytes[code_index] = 1;
            }
        }

        // Append the frame delimiter.
        tx_bytes[write_index++] = 0;

        // Start the DMA transfer.
        USART1->SR = 0;
        DMA1_Channel4->CNDTR = write_index;
        DMA1_Channel4->CCR |= DMA_CCR_EN;
    }
}

void swd_task(void *) {
    freertos::PeriodScheduler scheduler;
    while (true) {
        scheduler.delay_until_ms(1000);
        hal::swd_printf("--------------------------------\n");
        hal::swd_printf("Uptime: %u\n", freertos::uptime_ms() / 1000);

        const auto can_stats = can::get_stats();
        hal::swd_printf("CAN status: %s %u/%u %u/%u\n", can::is_online() ? "online" : "offline", can_stats.rx_count,
                        can_stats.lost_rx_count, can_stats.tx_count, can_stats.lost_tx_count);
    }
}

} // namespace

extern "C" void DMA1_Channel1_IRQHandler() {
    BaseType_t higher_priority_task_woken = pdFALSE;
    DMA1->IFCR |= DMA_IFCR_CTCIF1;
    vTaskNotifyGiveFromISR(*s_main_task, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

extern "C" void DMA1_Channel4_IRQHandler() {
    // Clear the pending flag and disable the channel.
    DMA1->IFCR |= DMA_IFCR_CTCIF4;
    DMA1_Channel4->CCR &= ~DMA_CCR_EN;
}

extern "C" void EXTI15_10_IRQHandler() {
    // Clear pending bit.
    EXTI->PR = EXTI_PR_PR10;

    // Notify expander task of brake switch change.
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(*s_expander_task, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

extern "C" void I2C1_EV_IRQHandler() {
    if (!s_i2c_sm.event()) {
        // State not changed.
        return;
    }

    const auto state = s_i2c_sm.state();
    if (state == i2c::State::Idle || state == i2c::State::NoAck || state == i2c::State::Error) {
        // Signal transaction completion.
        BaseType_t higher_priority_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(*s_expander_task, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

extern "C" void I2C1_ER_IRQHandler() {
    s_i2c_sm.error();
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(*s_expander_task, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

void vApplicationIdleHook() {
    hal::enter_sleep_mode(hal::WakeupSource::Interrupt);
}

void app_main() {
    s_radio_data.init();
    s_main_task.init(&main_task, "main", 4);
    s_expander_task.init(&expander_task, "expander", 2);
    s_radio_task.init(&radio_task, "radio", 1);
    if constexpr (config::enable_debug_logs()) {
        s_swd_task.init(&swd_task, "swd", 0);
    }
    vTaskStartScheduler();
}
