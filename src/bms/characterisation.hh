#pragma once

#include <cstddef>
#include <span>

namespace bms {

class CharacterisationRun {
public:
    enum class State {
        Start,
        Discharge,
        Rest,
        Done,
    };

private:
    const std::span<std::pair<float, float>> m_results;
    const float m_nominal_charge;
    const float m_stop_voltage;
    const float m_rest_time;
    const float m_delta_soc;

    State m_state{State::Start};
    float m_total_charge{0.0f};
    float m_step_charge{0.0f};
    float m_rest_timer{0.0f};
    std::size_t m_result_index{0};

public:
    CharacterisationRun(std::span<std::pair<float, float>> results, float nominal_charge, float stop_voltage,
                        float rest_time)
        : m_results(results), m_nominal_charge(nominal_charge), m_stop_voltage(stop_voltage), m_rest_time(rest_time),
          m_delta_soc(1.0f / (results.size() - 1)) {}

    void update(float dt, float current, float voltage);

    State state() const { return m_state; }
    std::size_t result_index() const { return m_result_index; }
    bool should_discharge() const { return m_state == State::Discharge; }
    bool is_done() const { return m_state == State::Done; }
};

} // namespace bms
