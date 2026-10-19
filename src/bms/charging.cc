#include <bms/charging.hh>

#include <algorithm>
#include <cstdint>
#include <limits>

namespace bms::charging {
namespace {

/**
 * @brief Cell target voltage setpoint in constant voltage phase in 100 uV resolution.
 */
constexpr std::uint16_t k_cv_cell_target = 41850;

} // namespace

Control Run::update(std::span<const std::uint16_t> cell_voltages, std::span<std::uint16_t> balance_bitsets,
                    const std::uint16_t charger_current, const bool running) {
    auto min_cell_voltage = std::numeric_limits<std::uint16_t>::max();
    auto max_cell_voltage = std::numeric_limits<std::uint16_t>::min();
    std::uint32_t pack_voltage = 0;
    for (const auto cell_voltage : cell_voltages) {
        if (cell_voltage != std::numeric_limits<std::uint16_t>::max()) {
            min_cell_voltage = std::min(min_cell_voltage, cell_voltage);
            max_cell_voltage = std::max(max_cell_voltage, cell_voltage);
            pack_voltage += cell_voltage;
        }
    }

    // Set a fixed 8 amp limit maximum.
    Control control{
        .target_current = static_cast<std::uint16_t>(min_cell_voltage > 30000 ? 80 : 20),
    };

    // If we are done make sure all cell balancers are disabled.
    if (m_state == State::Complete) {
        std::fill(balance_bitsets.begin(), balance_bitsets.end(), 0u);
        return control;
    }

    // Switch over to constant voltage when any cell hits 4.15 volts.
    if (max_cell_voltage >= 41500) {
        m_state = State::ConstantVoltage;
    }

    auto set_balancer_enabled = [&](std::size_t cell, bool enabled) {
        const auto bit = 1u << (cell % 16);
        if (enabled) {
            balance_bitsets[cell / 16] |= bit;
        } else {
            balance_bitsets[cell / 16] &= ~bit;
        }
    };

    if (m_state == State::ConstantCurrent) {
        // Set the max terminal voltage to a bit higher than the current pack voltage. Pack voltage is in 100 uV steps,
        // whereas the charger control is in 100 mV steps.
        control.target_voltage = static_cast<std::uint16_t>((pack_voltage + 150000) / 1000);
        control.enable = true;

        // Enable a very basic balancing control in constant current mode to try to counter badly mismatched cells
        // early.
        for (std::size_t cell = 0; cell < cell_voltages.size(); cell++) {
            if (cell_voltages[cell] == std::numeric_limits<std::uint16_t>::max()) {
                set_balancer_enabled(cell, false);
                continue;
            }

            if (cell_voltages[cell] >= min_cell_voltage + 500 && cell_voltages[cell] >= 37000) {
                set_balancer_enabled(cell, true);
            } else if (cell_voltages[cell] <= min_cell_voltage + 200) {
                set_balancer_enabled(cell, false);
            }
        }
        return control;
    }

    // Else we are in constant voltage control. Initialise the target voltage to the pack voltage minus a
    // fixed amount to smooth the transition into CV phase.
    const auto pack_target = static_cast<std::uint16_t>(pack_voltage / 1000);
    m_cv_target = m_cv_target.value_or(static_cast<std::uint16_t>(pack_target - 10));

    // Global balancer enable logic with hysteresis.
    const auto range = max_cell_voltage - min_cell_voltage;
    if (range >= 100) {
        m_balance_enable = true;
    } else if (range <= 50) {
        m_balance_enable = false;
    }

    // Balancer control.
    for (std::size_t cell = 0; cell < cell_voltages.size(); cell++) {
        if (!m_balance_enable || cell_voltages[cell] == std::numeric_limits<std::uint16_t>::max()) {
            set_balancer_enabled(cell, false);
            continue;
        }

        if (cell_voltages[cell] >= max_cell_voltage - 30) {
            set_balancer_enabled(cell, true);
        } else if (cell_voltages[cell] <= max_cell_voltage - 50) {
            set_balancer_enabled(cell, false);
        }
    }

    // Soft limit at 4.2 volts and back off the voltage by 1 volt until around 4.17 volts.
    if (!m_soft_tripped && max_cell_voltage >= 42000) {
        m_soft_tripped = true;
        if (m_cv_target && *m_cv_target >= 10) {
            *m_cv_target -= 10;
        }
    } else if (m_soft_tripped && max_cell_voltage <= k_cv_cell_target) {
        m_soft_tripped = false;
    }

    // Voltage target control.
    if (running && !m_soft_tripped) {
        const auto error = static_cast<std::int32_t>(max_cell_voltage) - k_cv_cell_target;
        if (error > 0 && m_holdoff_counter >= 10) {
            // Overshot the target slightly, so pull back the commanded target voltage.
            m_holdoff_counter = 0;
            *m_cv_target -= static_cast<std::uint16_t>(std::clamp<std::int32_t>(error / 20, 1, 10));
        } else if (error < -50 && m_holdoff_counter >= 30) {
            // Below target by 5 mV still, so step up the voltage slowly.
            m_holdoff_counter = 0;
            (*m_cv_target)++;
        }
        m_holdoff_counter++;

        // Keep the target close to the pack voltage (3% window). This avoids our target voltage drifting a lot in
        // case of the charger still being current limited, or if a high cell is taking a while to passively balance
        // down. The window needs to be big enough to account for any charger setpoint inaccuracy.
        const auto window = static_cast<std::uint16_t>(pack_target * 30 / 1000);
        *m_cv_target = std::clamp<std::uint16_t>(*m_cv_target, pack_target - window, pack_target + window);

        // TODO: We should track reported voltage by the charger against pack_voltage and potentially use the charger's
        //       voltage in some checks.
    }

    // Complete when all cells within 15 mV of the target and current is below 1.1 amps.
    if (running && min_cell_voltage >= k_cv_cell_target - 150 && charger_current <= 11) {
        m_complete_counter++;
    } else {
        m_complete_counter = 0;
    }
    if (m_complete_counter >= 300) {
        m_state = State::Complete;
        std::fill(balance_bitsets.begin(), balance_bitsets.end(), 0u);
        return Control{};
    }

    control.target_voltage = *m_cv_target;
    control.enable = !m_soft_tripped;
    return control;
}

} // namespace bms::charging
