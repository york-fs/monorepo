#pragma once

#include <cstdint>
#include <optional>
#include <span>

namespace bms::charging {

enum class State : std::uint8_t {
    ConstantCurrent,
    ConstantVoltage,
    Complete,
};

struct Control {
    std::uint16_t target_voltage{0};
    std::uint16_t target_current{0};
    bool enable{false};
};

class Run {
    State m_state{State::ConstantCurrent};
    std::optional<std::uint16_t> m_cv_target;
    std::uint32_t m_complete_timer{0};
    std::uint32_t m_holdoff_timer{0};
    bool m_balance_enable{false};
    bool m_soft_tripped{false};

public:
    Control update(std::span<const std::uint16_t> cell_voltages, std::span<std::uint16_t> balance_bitsets,
                   std::uint16_t charger_current, bool running);

    State state() const { return m_state; }
};

} // namespace bms::charging
