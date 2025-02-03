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
    std::atomic<std::int16_t> m_controller_temperature;
    std::atomic<std::int16_t> m_motor_temperature;
    std::atomic<bool> m_drive_enabled;

public:
    void operator()(const dti::GeneralData1 &) {}

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

    std::int16_t controller_temperature() const { return m_controller_temperature.load(std::memory_order_relaxed); }
    std::int16_t motor_temperature() const { return m_motor_temperature.load(std::memory_order_relaxed); }
    bool is_drive_enabled() const { return m_drive_enabled.load(std::memory_order_relaxed); }
} s_dti_state;

bool s_should_send_status = false;
bool s_should_update_dti = false;

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

void app_main() {
    // Initialise CAN peripheral and route all DTI messages to FIFO 0.
    can::init();
    can::route_filter(0, 0, 0x7feu, (config::k_dti_can_id << 3u) | 0b100u);

    // Install FIFO message pending callback and enable IRQ with a high priority.
    can::set_fifo_callback(0, [](const can::Message &message) {
        std::visit(s_dti_state, dti::parse_packet(message));
    });
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
    TIM2->ARR = 27999; // 2000 ms
    TIM3->ARR = 34;    // 2.5 ms

    // Enable timers and IRQs.
    TIM2->CR1 |= TIM_CR1_CEN;
    TIM3->CR1 |= TIM_CR1_CEN;
    hal::enable_irq(TIM2_IRQn, 4);
    hal::enable_irq(TIM3_IRQn, 3);

    // Main control loop.
    while (true) {
        if (std::exchange(s_should_send_status, false)) {
            // TODO: Send status report.
            hal::swd_printf("controller temp: %d, motor temp: %d, drive enabled: %s\n",
                            s_dti_state.controller_temperature(), s_dti_state.motor_temperature(),
                            s_dti_state.is_drive_enabled() ? "yes" : "no");
        }

        if (std::exchange(s_should_update_dti, false)) {
            // Set 250 RPM.
            can::transmit(dti::build_set_erpm(config::k_dti_can_id, 250 * config::k_erpm_factor));
        }
    }
}
