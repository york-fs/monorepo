#pragma once

#include <cstdint>
#include <optional>
#include <span>

namespace bms::charging {

enum class State : std::uint8_t {
    ConstantCurrent,
    ConstantVoltage,
    Complete,

    /**
     * @brief Stop requested. The charger is disabled but the contactors stay closed until its current has dropped.
     */
    Stopping,

    /**
     * @brief Stopped by request. The charger is disabled and the contactors are allowed to open.
     */
    Stopped,

    /**
     * @brief Ended because the BMS opened the shutdown circuit. Needs a new start to charge again.
     */
    Faulted,
};

/**
 * @brief Whether the charge is still running, i.e. the contactors should be held closed.
 */
constexpr bool is_active(State state) {
    return state == State::ConstantCurrent || state == State::ConstantVoltage || state == State::Stopping;
}

struct Control {
    std::uint16_t target_voltage{0};
    std::uint16_t target_current{0};
    bool enable{false};
};

class Run {
    State m_state;
    std::optional<std::uint16_t> m_cv_target;
    std::uint32_t m_complete_counter{0};
    std::uint32_t m_holdoff_counter{0};
    std::uint32_t m_stopping_counter{0};
    bool m_balance_enable{false};
    bool m_soft_tripped{false};

public:
    explicit Run(State initial_state = State::ConstantCurrent) : m_state(initial_state) {}

    Control update(std::span<const std::uint16_t> cell_voltages, std::span<std::uint16_t> balance_bitsets,
                   std::uint16_t charger_current, bool running);

    /**
     * @brief Requests a stop. Does nothing if the charge has already ended.
     */
    void request_stop();

    /**
     * @brief Ends the charge because of a BMS fault. Does nothing if the charge has already ended.
     */
    void fault();

    State state() const { return m_state; }
};

} // namespace bms::charging
