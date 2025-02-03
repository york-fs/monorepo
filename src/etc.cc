#include <can.hh>
#include <config.hh>
#include <dti.hh>
#include <hal.hh>
#include <stm32f103xb.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <utility>
#include <variant>

namespace {

class DtiState {
    std::atomic<std::int32_t> m_erpm{};
    std::atomic<std::int16_t> m_controller_temperature{};
    std::atomic<std::int16_t> m_motor_temperature{};
    std::atomic<bool> m_drive_enabled{};

public:
    void operator()(const dti::GeneralData1 &gd1) { m_erpm.store(gd1.erpm, std::memory_order_relaxed); }

    void operator()(const dti::GeneralData2 &) {}

    void operator()(const dti::GeneralData3 &gd3) {
        m_controller_temperature.store(gd3.controller_temperature, std::memory_order_relaxed);
        m_motor_temperature.store(gd3.motor_temperature, std::memory_order_relaxed);
    }

    void operator()(const dti::GeneralData5 &gd5) {
        m_drive_enabled.store(gd5.drive_enabled, std::memory_order_relaxed);
    }

    void operator()(const dti::UnknownMessageType &) {
        // TODO: Log this as a warning.
    }

    std::int32_t erpm() const { return m_erpm.load(std::memory_order_relaxed); }
    std::int16_t controller_temperature() const { return m_controller_temperature.load(std::memory_order_relaxed); }
    std::int16_t motor_temperature() const { return m_motor_temperature.load(std::memory_order_relaxed); }
    bool is_drive_enabled() const { return m_drive_enabled.load(std::memory_order_relaxed); }
} s_dti_state;

struct CalibrationData {
    std::array<std::uint16_t, 100> ring_buffer{};
    std::uint32_t ring_index{};
    std::uint16_t max_value{};
    std::uint16_t min_value{UINT16_MAX};

    std::uint32_t update(std::uint16_t adc_value);
};

enum class LedState : std::uint32_t {
    Off = 0u,
    Calibrating,
    Uncalibrated,
    SensorError,
    On,
};

enum class State {
    Uncalibrated,
    Calibrating,
    CalibrationWait,
    Running,
};

std::array<std::uint16_t, 2> s_adc_buffer{};
std::array<std::uint32_t, static_cast<std::uint32_t>(LedState::On) * 2u> s_led_dma{};

CalibrationData s_left_calibration;
CalibrationData s_right_calibration;
std::atomic<LedState> s_led_state{LedState::Off};
std::atomic<State> s_state{State::Uncalibrated};

const char *state_name(State state) {
    switch (state) {
    case State::Uncalibrated:
        return "uncalibrated";
    case State::Calibrating:
        return "calibrating";
    case State::CalibrationWait:
        return "calibration wait";
    case State::Running:
        return "running";
    }
    return "unknown";
}

void set_led_state(LedState state) {
    if (s_led_state.exchange(state) == state) {
        return;
    }

    // Disable DMA channel.
    DMA1_Channel7->CCR &= ~DMA_CCR_EN;

    // Handle special on case.
    if (state == LedState::On) {
        s_led_dma[0] = 1u << 13u;
        DMA1_Channel7->CNDTR = 1;
        DMA1_Channel7->CCR |= DMA_CCR_EN;
        return;
    }

    const auto count = static_cast<std::uint32_t>(state);
    for (std::uint32_t i = 0; i < count; i++) {
        s_led_dma[i * 2] = 1u << 13u;
        s_led_dma[i * 2 + 1] = 1u << 29u;
    }
    s_led_dma[count * 2] = 1u << 29u;
    s_led_dma[count * 2 + 1] = 1u << 29u;

    // Re-enable DMA channel.
    DMA1_Channel7->CNDTR = count * 2u + 2u;
    DMA1_Channel7->CCR |= DMA_CCR_EN;
}

std::uint32_t CalibrationData::update(std::uint16_t adc_value) {
    max_value = std::max(max_value, adc_value);
    ring_buffer[ring_index] = adc_value;
    ring_index = (ring_index + 1) % ring_buffer.size();

    std::uint32_t average = 0;
    for (std::uint16_t value : ring_buffer) {
        average += static_cast<std::uint32_t>(value);
    }
    return average / ring_buffer.size();
}

bool calibration_iteration() {
    const auto left_now = static_cast<std::int32_t>(s_adc_buffer[0]);
    const auto right_now = static_cast<std::int32_t>(s_adc_buffer[1]);
    const auto left_average = s_left_calibration.update(s_adc_buffer[0]);
    const auto right_average = s_right_calibration.update(s_adc_buffer[1]);

    // TODO: Only using left sensor here. When to do discrepancy detection?
    if (std::abs(left_now - s_left_calibration.max_value) < 200) {
        // Pedal hasn't moved enough from start position - don't finish calibration.
        return false;
    }

    if (std::abs(left_now - static_cast<std::int32_t>(left_average)) > 10) {
        // Pedal still moving - don't finish calibration.
        return false;
    }

    // Calibration complete - store min value.
    for (std::uint16_t value : s_left_calibration.ring_buffer) {
        s_left_calibration.min_value = std::min(s_left_calibration.min_value, value);
    }
    return true;
}

std::uint16_t calculate_current() {
    // TODO: Use a lookup table.
    std::int32_t x = s_adc_buffer[0] - s_left_calibration.min_value;
    float normalised = static_cast<float>(x) / (s_left_calibration.max_value - s_left_calibration.min_value);
    normalised = 1.0f - normalised;
    float curve = 1.0f / (1.0f + std::exp(-10.0f * (normalised - 0.5f)));
    auto current = static_cast<std::uint16_t>(curve * 1000.0f);
    if (current < 20) {
        return 0;
    }

    // Apply current preload if needed.
    if (s_dti_state.erpm() < 1000) {
        current += 100;
    }
    return current;
}

} // namespace

extern "C" void EXTI15_10_IRQHandler() {
    // Clear all pending interrupts just in case.
    const auto pending = std::exchange(EXTI->PR, 0x7ffffu);
    if ((pending & EXTI_PR_PR14) != 0u) {
        s_left_calibration = {};
        s_right_calibration = {};
        s_state.store(State::Calibrating);
    }
}

extern "C" void TIM2_IRQHandler() {
    // Clear update interrupt flag.
    TIM2->SR = ~TIM_SR_UIF;
    // TODO: Send proper status report over CAN.
    hal::swd_printf("State: %s\n", state_name(s_state.load()));
}

extern "C" void TIM3_IRQHandler() {
    // Clear update interrupt flag.
    TIM3->SR = ~TIM_SR_UIF;

    switch (s_state.load()) {
    case State::Uncalibrated:
        set_led_state(LedState::Uncalibrated);
        break;
    case State::Calibrating:
        set_led_state(LedState::Calibrating);
        if (calibration_iteration()) {
            s_state.store(State::CalibrationWait);
        }
        break;
    case State::CalibrationWait:
        set_led_state(LedState::On);
        if (std::abs(static_cast<std::int32_t>(s_adc_buffer[0]) -
                     static_cast<std::int32_t>(s_left_calibration.max_value)) < 10) {
            s_state.store(State::Running);
        }
        break;
    case State::Running:
        set_led_state(LedState::Off);
        const auto current = calculate_current();
        hal::swd_printf("Current: %u, ERPM: %d\n", current, s_dti_state.erpm());
        can::transmit(dti::build_set_relative_current(config::k_dti_can_id, static_cast<std::int16_t>(current)));
        break;
    }

    // Queue next ADC read.
    hal::adc_start(ADC1);
}

void app_main() {
    // Initialise CAN peripheral and route all DTI messages to FIFO 0.
    can::init();
    can::route_filter(0, 0, 0x7feu, (config::k_dti_can_id << 3u) | 0b100u);

    // Install FIFO message pending callback and enable IRQ with a high priority.
    can::set_fifo_callback(0, [](const can::Message &message) {
        std::visit(s_dti_state, dti::parse_packet(message));
    });
    hal::enable_irq(USB_LP_CAN1_RX0_IRQn, 2);

    // Configure GPIOs for ADC channels.
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    hal::configure_gpio(GPIOA, 0, hal::GpioInputMode::Analog);
    hal::configure_gpio(GPIOA, 1, hal::GpioInputMode::Analog);

    // Configure GPIOs for LED and button.
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    hal::configure_gpio(GPIOB, 13, hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max10);
    hal::configure_gpio(GPIOB, 14, hal::GpioInputMode::PullUpPullDown);
    GPIOB->ODR |= 1u << 14u;

    // Enable timer 2 and 3 clocks.
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;

    // Enable update event interrupt/DMA request generation.
    TIM2->DIER |= TIM_DIER_UIE;
    TIM3->DIER |= TIM_DIER_UIE;
    TIM4->DIER |= TIM_DIER_UDE;

    // Configure 1000 ms timer for status reports.
    TIM2->PSC = 7999;
    TIM2->ARR = 6999;

    // Configure 10 ms timer for main throttle updates.
    TIM3->PSC = 249;
    TIM3->ARR = 2239;

    // Configure timer for LED DMA.
    TIM4->PSC = 1999;
    TIM4->ARR = 3999;

    // Enable timers and IRQs.
    TIM2->CR1 |= TIM_CR1_CEN;
    TIM3->CR1 |= TIM_CR1_CEN;
    TIM4->CR1 |= TIM_CR1_CEN;
    hal::enable_irq(TIM2_IRQn, 4);
    hal::enable_irq(TIM3_IRQn, 3);

    // Configure DMA channel for LED.
    DMA1_Channel7->CPAR = std::bit_cast<std::uint32_t>(&GPIOB->BSRR);
    DMA1_Channel7->CMAR = std::bit_cast<std::uint32_t>(s_led_dma.data());
    DMA1_Channel7->CCR = DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_DIR;

    // Set 13.5 cycle conversion time (~1.85 us).
    hal::adc_init(ADC1, 2);
    hal::adc_init_dma(ADC1, DMA1_Channel1, s_adc_buffer);
    hal::adc_sequence_channel(ADC1, 1, 0, 0b010u);
    hal::adc_sequence_channel(ADC1, 2, 1, 0b010u);

    AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI14_PB;
    EXTI->IMR |= EXTI_IMR_MR14;
    EXTI->FTSR |= EXTI_FTSR_TR14;
    hal::enable_irq(EXTI15_10_IRQn, 5);

    // TODO: Synchronise timers together.

    while (true) {
        // TODO: WFI.
    }
}
