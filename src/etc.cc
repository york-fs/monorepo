#include <can.hh>
#include <config.hh>
#include <dti.hh>
#include <hal.hh>
#include <stm32f103xb.h>

#include <atomic>
#include <cstdint>
#include <utility>
#include <variant>

namespace {

class DtiState {
    // TODO: Record more data such as actual RPM to crosscheck it.
    std::atomic<bool> m_drive_enabled;

public:
    void operator()(const dti::GeneralData1 &gd1) {}

    void operator()(const dti::GeneralData2 &) {}

    void operator()(const dti::GeneralData3 &) {}

    void operator()(const dti::GeneralData5 &gd5) { m_drive_enabled.store(true, std::memory_order_relaxed); }

    void operator()(const dti::UnknownData &) {
        // TODO: Log this as a warning.
    }

    bool is_drive_enabled() const { return m_drive_enabled.load(std::memory_order_relaxed); }
} s_dti_state;

bool s_should_send_status = false;
bool s_should_update_dti = false;

void dti_message_callback(const can::Message &message) {
    const auto dti_packet = dti::parse_packet(message.identifier, message.data_low, message.data_high);
    std::visit(s_dti_state, dti_packet);
}

void update_dti() {
    // TODO: Compute based on RTD and inverter data sanity checking.
    bool drive_enabled_desired = true;

    // Inverter's drive enable state doesn't match what we want, attempt to update it.
    if (s_dti_state.is_drive_enabled() != drive_enabled_desired) {
        // Send drive enable message.
        can::Message message{
            .identifier = (0x0cul << 8u) | config::k_dti_can_id,
            .data_low = drive_enabled_desired ? 0x1u : 0x0u,
            .length = 1,
        };
        if (!can::transmit(message)) {
            // TODO: Handle this?
        }

        // Early return so we avoid sending throttle data when drive is not enabled.
        return;
    }

    // TODO: Calculate desired RPM based on throttle pedal input.
    std::int32_t desired_rpm = 500;

    // TODO: Sanity check RPM.

    // Send set speed (ERPM) message.
    can::Message message{
        .identifier = (0x03ul << 8u) | config::k_dti_can_id,
        .data_low = static_cast<std::uint32_t>(desired_rpm * config::k_erpm_factor),
        .length = 4,
    };
    if (!can::transmit(message)) {
        // TODO: Handle this?
    }
}

} // namespace

extern "C" void TIM2_IRQHandler() {
    // Clear update interrupt flag.
    TIM2->SR = ~TIM_SR_UIF;
    s_should_send_status = true;
}

extern "C" void TIM3_IRQHandler() {
    // Clear update interrupt flag.
    TIM3->SR = ~TIM_SR_UIF;
    s_should_update_dti = true;
}

int main() {
    hal::init_clocks();

    // Initialise CAN peripheral and route all DTI messages to FIFO 0.
    can::init();
    can::route_filter(0, 0, 0x7feu, (config::k_dti_can_id << 3u) | 0b100u);

    // Install FIFO message pending callback and enable IRQ with a high priority.
    can::set_fifo_callback(0, dti_message_callback);
    hal::enable_irq(USB_LP_CAN1_RX0_IRQn, 2);

    // Enable timer 2 and 3 clocks.
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;

    // Enable update event interrupt generation.
    TIM2->DIER |= TIM_DIER_UIE;
    TIM3->DIER |= TIM_DIER_UIE;

    // Configure prescaler.
    TIM2->PSC = 1999;
    TIM3->PSC = 1999;

    // Configure auto reload registers.
    TIM2->ARR = 15999;
    TIM3->ARR = 7999;

    // Enable timers and IRQs.
    TIM2->CR1 |= TIM_CR1_CEN;
    TIM3->CR1 |= TIM_CR1_CEN;
    hal::enable_irq(TIM2_IRQn, 4);
    hal::enable_irq(TIM3_IRQn, 3);

    // Main control loop.
    while (true) {
        if (std::exchange(s_should_send_status, false)) {
            // TODO: Send status report.
        }

        if (std::exchange(s_should_update_dti, false)) {
            update_dti();
        }

        // Sleep until next interrupt.
        __WFI();
    }
}
