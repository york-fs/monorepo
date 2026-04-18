#include <bms/characterisation.hh>

#include <utility>

namespace bms {

void CharacterisationRun::update(float dt, float current, float voltage) {
    if (m_state == State::Start) {
        m_results[m_result_index++] = std::make_pair(0.0f, voltage);
        m_state = voltage > m_stop_voltage ? State::Discharge : State::Done;
    } else if (m_state == State::Discharge) {
        const float dq = current * dt / 3600.0f;
        m_step_charge += dq;
        m_total_charge += dq;
        if (m_step_charge >= m_delta_soc * m_nominal_charge || voltage <= m_stop_voltage) {
            m_state = State::Rest;
            m_step_charge = 0.0f;
            m_rest_timer = 0.0f;
        }
    } else if (m_state == State::Rest) {
        m_rest_timer += dt;
        if (m_rest_timer >= m_rest_time) {
            m_results[m_result_index++] = std::make_pair(m_total_charge, voltage);
            m_state = (voltage > m_stop_voltage && m_result_index < m_results.size()) ? State::Discharge : State::Done;
        }
    }
}

} // namespace bms
