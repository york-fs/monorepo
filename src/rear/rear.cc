#include <can.hh>
#include <config.hh>
#include <dti.hh>
#include <freertos.hh>
#include <front/can_messages.hh>
#include <front/shutdown.hh>
#include <hal.hh>
#include <i2c.hh>
#include <precharge/can_messages.hh>
#include <precharge/state.hh>
#include <rear/can_messages.hh>
#include <rear/fuse.hh>
#include <rear/online.hh>
#include <rear/shutdown.hh>
#include <stm32f103xb.h>
#include <time_tracked.hh>
#include <util/flag_bitset.hh>
#include <util/stream.hh>
#include <util/type_traits.hh>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <span>

using namespace rear;

namespace {

/**
 * @brief Control task scheduling period in milliseconds.
 */
constexpr std::uint32_t k_control_period = 10;

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

enum class RearShutdownSample : std::uint16_t {
    Latch = 0,
    RightEstop = 1,
    Hvd = 2,
    FrontOutput = 3,
    Aux = 4,
    Tsms = 5,
    LeftEstop = 6,
    Bspd = 7,
    BmsOk = 8,
    DtiOk = 9,
    ImdOk = 10,
};

using RearShutdownSamples = util::FlagBitset<RearShutdownSample>;

struct RadioData {
    OnlineFlags online_flags;
    std::optional<precharge::StatusMessage> precharge_status;
    FuseBitset fuse_bitset;
    std::uint16_t lvs_min_voltage;
    std::uint16_t lvs_max_voltage;
    ShutdownCircuitOpenCause shutdown_open_cause;
    TsPreventionFlags ts_prevention_flags;
    RtdPreventionFlags rtd_prevention_flags;
};

// Single-entry queue for consistent radio data.
freertos::Queue<RadioData, 1> s_radio_data;

// I2C state machine for GPIO expander.
i2c::StateMachine s_i2c_sm(i2c::Bus::_1, i2c::Speed::_100);

freertos::Task<256> s_control_task;
freertos::Task<256> s_radio_task;
freertos::Task<128> s_swd_task;

ShutdownCircuitOpenCause compute_shutdown_open_cause(const std::optional<front::StatusMessage> &front_status,
                                                     RearShutdownSamples rear_samples) {
    // Shutdown input from LVMS and BSPD.
    if (rear_samples.is_clear(RearShutdownSample::Bspd)) {
        // TODO: We currently can't detect whether shutdown is present before the BSPD or not, hence we return a generic
        // rear shutdown input reason instead of BSPD.
        return ShutdownCircuitOpenCause::RearInput;
    }

    // Front.
    if (!front_status) {
        return ShutdownCircuitOpenCause::FrontOutput;
    }
    if (front_status->shutdown_samples.is_clear(front::ShutdownSample::EmergencyStop)) {
        return ShutdownCircuitOpenCause::FrontEstop;
    }
    if (front_status->shutdown_samples.is_clear(front::ShutdownSample::BrakeOverTravel)) {
        return ShutdownCircuitOpenCause::BrakeOverTravel;
    }
    if (front_status->shutdown_samples.is_clear(front::ShutdownSample::InertiaSwitch)) {
        return ShutdownCircuitOpenCause::InertiaSwitch;
    }
    if (front_status->shutdown_samples.is_clear(front::ShutdownSample::Auxiliary)) {
        return ShutdownCircuitOpenCause::FrontAuxiliary;
    }
    if (rear_samples.is_clear(RearShutdownSample::FrontOutput)) {
        // Signal not getting from front even though over CAN, front is reporting all ok.
        return ShutdownCircuitOpenCause::FrontOutput;
    }

    // Shutdown latch.
    if (rear_samples.is_clear(RearShutdownSample::BmsOk)) {
        return ShutdownCircuitOpenCause::BmsLatch;
    }
    if (rear_samples.is_clear(RearShutdownSample::ImdOk)) {
        return ShutdownCircuitOpenCause::ImdLatch;
    }
    if (rear_samples.is_clear(RearShutdownSample::DtiOk)) {
        return ShutdownCircuitOpenCause::InverterInterlock;
    }
    if (rear_samples.is_clear(RearShutdownSample::Latch)) {
        return ShutdownCircuitOpenCause::ShutdownLatchFailure;
    }

    // Rear E-stops.
    if (rear_samples.is_clear(RearShutdownSample::LeftEstop)) {
        return ShutdownCircuitOpenCause::LeftEstop;
    }
    if (rear_samples.is_clear(RearShutdownSample::RightEstop)) {
        return ShutdownCircuitOpenCause::RightEstop;
    }

    // High voltage disconnect interlock.
    if (rear_samples.is_clear(RearShutdownSample::Hvd)) {
        return ShutdownCircuitOpenCause::HvdInterlock;
    }

    // Shutdown on rear auxiliary connector.
    if (rear_samples.is_clear(RearShutdownSample::Aux)) {
        return ShutdownCircuitOpenCause::RearAuxiliary;
    }

    // Tractive system master switch.
    if (rear_samples.is_clear(RearShutdownSample::Tsms)) {
        return ShutdownCircuitOpenCause::Tsms;
    }
    return ShutdownCircuitOpenCause::None;
}

bool is_precharge_state_good(precharge::State state) {
    switch (state) {
    case precharge::State::Standby:
    case precharge::State::Precharge:
    case precharge::State::PrechargeHold:
    case precharge::State::Active:
        return true;
    default:
        return false;
    }
}

bool expander_wait() {
    const bool timeout = freertos::notify_take(1, true, pdMS_TO_TICKS(2)) == 0;
    const auto state = s_i2c_sm.state();
    if (!timeout && (state == i2c::State::Idle || state == i2c::State::NoAck)) {
        return state == i2c::State::Idle;
    }
    // Reset the I2C peripheral.
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

/**
 * @brief The control task is the main coordination task of the rear distribution, and therefore of the whole car.
 *
 * The task's operation can be summarised as doing the following:
 * - Takes in data over CAN from all other components of the car.
 * - Samples local signals such as fuse voltages, series shutdown levels, the IMD, and brake switch signals.
 * - Derives fuse statuses and shutdown circuit state.
 * - Decides whether the TS and RTD states can be active.
 * - Sends control messages to the precharge and inverter.
 */
void control_task(void *) {
    // Initialise CAN on port B.
    can::init(can::Port::B, config::k_can_speed, 3);

    // Data from the front distribution.
    static TimeTracked<front::StatusMessage> front_status(250);
    static TimeTracked<front::ThrottleMessage> front_throttle(25);
    static std::array<std::uint16_t, 7> front_lvs_voltages{};

    // Data from the precharge board.
    static TimeTracked<precharge::StatusMessage> precharge_status(25);

    // Setup CAN listeners.
    can::listen<front::StatusMessage, [](const front::StatusMessage &message) {
        front_status.receive(message);
    }>(config::k_front_can_id, 0);
    can::listen<front::ThrottleMessage, [](const front::ThrottleMessage &message) {
        front_throttle.receive(message);
    }>(config::k_front_can_id, 1);
    can::listen<front::LvsSampleMessage1, [](const front::LvsSampleMessage1 &message) {
        front_lvs_voltages[0] = message.rtd_voltage;
        front_lvs_voltages[1] = message.apps_1_voltage;
        front_lvs_voltages[2] = message.apps_2_voltage;
        front_lvs_voltages[3] = message.front_voltage;
    }>(config::k_front_can_id, 2);
    can::listen<front::LvsSampleMessage2, [](const front::LvsSampleMessage2 &message) {
        front_lvs_voltages[4] = message.dwin_voltage;
        front_lvs_voltages[5] = message.aux_1_voltage;
        front_lvs_voltages[6] = message.aux_2_voltage;
    }>(config::k_front_can_id, 3);
    can::listen<precharge::StatusMessage, [](const precharge::StatusMessage &message) {
        precharge_status.receive(message);
    }>(config::k_precharge_can_id, 4);

    // Initialise ADC to sample all LVS inputs.
    std::array<std::uint16_t, 10> adc_buffer{};
    hal::adc_init(ADC1, adc_buffer.size());
    for (std::uint32_t i = 0; i < adc_buffer.size(); i++) {
        hal::adc_sequence_channel(ADC1, i + 1, i, 0b010u);
    }
    hal::adc_init_dma(adc_buffer);
    DMA1_Channel1->CCR |= DMA_CCR_TCIE;

    // Configure expander I2C pins for peripheral use and initialise the state machine.
    hal::Gpio scl(hal::GpioPort::B, 6);
    hal::Gpio sda(hal::GpioPort::B, 7);
    scl.configure(hal::GpioOutputMode::AlternateOpenDrain, hal::GpioOutputSpeed::Max2);
    sda.configure(hal::GpioOutputMode::AlternateOpenDrain, hal::GpioOutputSpeed::Max2);
    s_i2c_sm.init();

    // Configure the DTI_OK and brake switch sample pins which are not on the expander.
    hal::Gpio dti_ok_sample(hal::GpioPort::B, 2);
    hal::Gpio brake_switch(hal::GpioPort::B, 10);
    dti_ok_sample.configure(hal::GpioInputMode::Floating);
    brake_switch.configure(hal::GpioInputMode::Floating);

    // Enable CAN, ADC, and I2C IRQs.
    hal::irq_enable(CAN1_SCE_IRQn, 5);
    hal::irq_enable(CAN1_TX_IRQn, 6);
    hal::irq_enable(CAN1_RX0_IRQn, 7);
    hal::irq_enable(DMA1_Channel1_IRQn, 8);
    hal::irq_enable(I2C1_EV_IRQn, 9);
    hal::irq_enable(I2C1_ER_IRQn, 9);

    // Configure the expander pins.
    expander_write(ExpanderRegister::OutputPort0, 0);
    expander_write(ExpanderRegister::OutputPort1, 0);
    expander_write(ExpanderRegister::PolarityPort0, 0);
    expander_write(ExpanderRegister::PolarityPort1, 0);
    expander_write(ExpanderRegister::ConfigurationPort0, 0xff);
    expander_write(ExpanderRegister::ConfigurationPort1, 0xee);

    // Keep track of whether the ready-to-drive state has been latched on. This is needed because, whilst most of the
    // RTD prevention flags are continuously checked, some, like the brake being pressed are only needed for initial
    // activation.
    bool rtd_latched = false;

    freertos::PeriodScheduler scheduler;
    while (true) {
        scheduler.delay_until_ms(k_control_period);

        // Update message expiry detections for all of the important time tracked messages.
        front_status.update();
        precharge_status.update();

        // Build a bitset of component online states.
        OnlineFlags online_flags;
        if (front_status) {
            online_flags.set(OnlineFlag::FrontOnline);
        }
        if (precharge_status) {
            online_flags.set(OnlineFlag::PrechargeOnline);
        }

        // Sample all ADC channels.
        hal::adc_start(ADC1);
        freertos::notify_take(0, true, portMAX_DELAY);

        // Create an array of rear and front measured fuse voltages. The sampling inputs have a 5.7x divider on them.
        std::array<std::uint16_t, 17> fuse_voltages{};
        std::transform(adc_buffer.begin(), adc_buffer.end(), fuse_voltages.begin(), [](std::uint16_t adc_value) {
            return (((k_mcu_vref * adc_value) >> 12) * 57) / 10;
        });
        if (front_status) {
            freertos::in_critical_section([&] {
                auto out_it = std::next(fuse_voltages.begin(), 10);
                std::copy(front_lvs_voltages.begin(), front_lvs_voltages.end(), out_it);
            });
        }

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

        // Sample rear-local shutdown pins.
        const auto expander_port_0 = expander_read(ExpanderRegister::InputPort0).value_or(0);
        const auto expander_port_1 = expander_read(ExpanderRegister::InputPort1).value_or(0);
        auto rear_shutdown_samples = RearShutdownSamples(expander_port_0);
        if ((expander_port_1 & (1u << 2)) != 0) {
            rear_shutdown_samples.set(RearShutdownSample::BmsOk);
        }
        if ((expander_port_1 & (1u << 3)) != 0) {
            rear_shutdown_samples.set(RearShutdownSample::ImdOk);
        }
        if (dti_ok_sample.read()) {
            rear_shutdown_samples.set(RearShutdownSample::DtiOk);
        }

        // Compute a cause for the shutdown circuit being open.
        const auto shutdown_open_cause = compute_shutdown_open_cause(front_status, rear_shutdown_samples);

        // Compute TS activation prevention flags.
        // TODO: Add BMS and inverter checks.
        TsPreventionFlags ts_prevention_flags;
        if (shutdown_open_cause != ShutdownCircuitOpenCause::None) {
            ts_prevention_flags.set(TsPreventionFlag::ShutdownOpen);
        }
        if (fuse_bitset.set_count() != fuse_voltages.size()) {
            ts_prevention_flags.set(TsPreventionFlag::BadFuse);
        }
        if (online_flags.is_clear(OnlineFlag::FrontOnline)) {
            ts_prevention_flags.set(TsPreventionFlag::FrontOffline);
        }
        if (!front_status || !front_status->ts_activation_desired) {
            ts_prevention_flags.set(TsPreventionFlag::NotRequested);
        }
        if (online_flags.is_clear(OnlineFlag::PrechargeOnline)) {
            ts_prevention_flags.set(TsPreventionFlag::PrechargeOffline);
        }
        if (!precharge_status || !is_precharge_state_good(precharge_status->state)) {
            ts_prevention_flags.set(TsPreventionFlag::PrechargeState);
        }

        // Compute RTD prevention flags.
        // TODO: Add APPS checks.
        RtdPreventionFlags rtd_prevention_flags;
        if (ts_prevention_flags.any_set()) {
            rtd_prevention_flags.set(RtdPreventionFlag::TsNotActive);
        }
        if (!precharge_status || precharge_status->state != precharge::State::Active) {
            rtd_prevention_flags.set(RtdPreventionFlag::TsNotActive);
        }
        if (!front_status || !front_status->rtd_activation_desired) {
            rtd_prevention_flags.set(RtdPreventionFlag::NotRequested);
        }
        rtd_latched &= rtd_prevention_flags.none_set();

        // Brake switch RTD flag is latched since it's only required when activating RTD.
        const bool brake_pressed = brake_switch.read();
        if (!rtd_latched && !brake_pressed) {
            rtd_prevention_flags.set(RtdPreventionFlag::BrakeNotPressed);
        }

        // Latch RTD active if no prevention flags set.
        if (rtd_prevention_flags.none_set()) {
            rtd_latched = true;
        }

        // Always set zero max brake current since no regen support for now.
        dti::SetMaxBrakeDirectCurrentMessage set_max_charge{
            .current = 0,
        };
        can::transmit(config::k_dti_can_id, set_max_charge);

        // Cut all power to the inverter if any flags are present preventing TS activation. If this happens, the
        // precharge allows an approximately 250 ms window for the heartbeat to expire before opening the AIRs. That
        // gives us a window to cut power smoothly before it's abruptly removed from the inverter.
        if (ts_prevention_flags.any_set()) {
            dti::SetMaxDirectCurrentMessage set_max_discharge{
                .current = 0,
            };
            can::transmit(config::k_dti_can_id, set_max_discharge);
        } else {
            // Good to set 200 amp discharge limit.
            // TODO: Get this from the BMS.
            dti::SetMaxDirectCurrentMessage set_max_discharge{
                .current = 2000,
            };
            can::transmit(config::k_dti_can_id, set_max_discharge);

            // Send precharge heatbeat.
            can::transmit(config::k_precharge_can_id, precharge::HeartbeatMessage{});
        }

        // Always send a throttle to the inverter to avoid it coasting. If RTD flags are present (including if TS is
        // off), we send zero absolute current. If the brake is pressed, we avoid powering the motor briefly.
        if (rtd_latched && front_throttle && !brake_pressed) {
            dti::SetRelativeCurrentMessage set_relative_current{
                .percentage = util::clamp(static_cast<std::int16_t>(front_throttle->desired_throttle), 0, 1000),
            };
            can::transmit(config::k_dti_can_id, set_relative_current);
        } else {
            dti::SetCurrentMessage set_current{
                .current = 0,
            };
            can::transmit(config::k_dti_can_id, set_current);
        }

        // Broadcast status message.
        StatusMessage status_message{
            .shutdown_open_cause = shutdown_open_cause,
            .ts_prevention_flags = ts_prevention_flags,
            .rtd_prevention_flags = rtd_prevention_flags,
        };
        can::transmit(config::k_rear_can_id, status_message);

        // Write to GPIO expander outputs.
        std::uint8_t expander_out = 0;
        if (brake_pressed) {
            expander_out |= 1u << 4;
        }
        expander_write(ExpanderRegister::OutputPort1, expander_out);

        // Update radio data.
        s_radio_data.overwrite(RadioData{
            .online_flags = online_flags,
            .precharge_status = precharge_status,
            .fuse_bitset = fuse_bitset,
            .lvs_min_voltage = lvs_min_voltage,
            .lvs_max_voltage = lvs_max_voltage,
            .shutdown_open_cause = shutdown_open_cause,
            .ts_prevention_flags = ts_prevention_flags,
            .rtd_prevention_flags = rtd_prevention_flags,
        });
    }
}

void radio_task(void *) {
    // Delay to allow the radio to leave its bootloader.
    vTaskDelay(pdMS_TO_TICKS(50));

    // Configure GPIOs.
    hal::Gpio radio_tx(hal::GpioPort::A, 9);
    hal::Gpio radio_rx(hal::GpioPort::A, 10);
    hal::Gpio radio_cts(hal::GpioPort::A, 11);
    hal::Gpio radio_rts(hal::GpioPort::A, 12);
    radio_tx.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max10);
    radio_rx.configure(hal::GpioInputMode::Floating);
    radio_cts.configure(hal::GpioInputMode::Floating);
    radio_rts.configure(hal::GpioInputMode::PullDown);

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

    // Keep track of latest valid messages received.
    precharge::StatusMessage precharge_status;

    freertos::PeriodScheduler scheduler;
    while (true) {
        scheduler.delay_until_ms(k_radio_period);

        // Don't transmit if the radio's UART buffer is near full.
        if (radio_cts.read()) {
            missed_tx_count++;
            continue;
        }

        const auto data = *s_radio_data.receive(portMAX_DELAY);
        if (data.precharge_status) {
            precharge_status = *data.precharge_status;
        }

        // Build a telemetry frame with the data offset by one byte to allow for the first COBS code. The size is also
        // limited to 3 bytes fewer to allow for the overall COBS overhead.
        util::Stream stream(std::span<std::uint8_t>(tx_bytes).subspan(1, 253));

        // Append general information.
        stream.write_be(freertos::uptime_ms());
        stream.write_byte(missed_tx_count);

        // Append online status bitset for each component.
        stream.write_be(data.online_flags.value());

        // Append distribution information.
        stream.write_be(data.fuse_bitset.value());
        stream.write_be(data.lvs_min_voltage);
        stream.write_be(data.lvs_max_voltage);
        stream.write_be(util::to_underlying(data.shutdown_open_cause));
        stream.write_be(data.ts_prevention_flags.value());
        stream.write_be(data.rtd_prevention_flags.value());

        // Append precharge information.
        stream.write_be(util::to_underlying(precharge_status.state));
        stream.write_be(precharge_status.error_flags.value());
        stream.write_be(precharge_status.precharge_voltage);
        stream.write_be(precharge_status.tractive_voltage);
        stream.write_be(precharge_status.relay_states.value());

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
    freertos::InterruptYielder interrupt_yielder;
    DMA1->IFCR |= DMA_IFCR_CTCIF1;
    s_control_task.notify_give_isr(0, interrupt_yielder);
}

extern "C" void DMA1_Channel4_IRQHandler() {
    // Clear the pending flag and disable the channel.
    DMA1->IFCR |= DMA_IFCR_CTCIF4;
    DMA1_Channel4->CCR &= ~DMA_CCR_EN;
}

extern "C" void I2C1_EV_IRQHandler() {
    freertos::InterruptYielder interrupt_yielder;
    if (!s_i2c_sm.event()) {
        // State not changed.
        return;
    }

    const auto state = s_i2c_sm.state();
    if (state == i2c::State::Idle || state == i2c::State::NoAck || state == i2c::State::Error) {
        // Signal transaction completion.
        s_control_task.notify_give_isr(1, interrupt_yielder);
    }
}

extern "C" void I2C1_ER_IRQHandler() {
    freertos::InterruptYielder interrupt_yielder;
    s_i2c_sm.error();
    s_control_task.notify_give_isr(1, interrupt_yielder);
}

void vApplicationIdleHook() {
    hal::enter_sleep_mode(hal::WakeupSource::Interrupt);
}

void app_main() {
    s_radio_data.init();
    s_control_task.init(&control_task, "main", 4);
    s_radio_task.init(&radio_task, "radio", 1);
    if constexpr (config::enable_debug_logs()) {
        s_swd_task.init(&swd_task, "swd", 0);
    }
    vTaskStartScheduler();
}
