#pragma once

#include <freertos.hh>

#include <cstdint>

namespace rear {

template <typename T>
class TimeTracked {
    TickType_t m_last_ticks{0};
    T m_data;

public:
    bool has_elapsed(std::uint32_t duration) const;
    void receive(const T &data);

    const T &operator*() const { return m_data; }
    const T *operator->() const { return &m_data; }
};

template <typename T>
bool TimeTracked<T>::has_elapsed(std::uint32_t duration) const {
    // TODO: Make helper in freertos.hh and merge with code in BMS.
    return xTaskGetTickCount() - m_last_ticks >= pdMS_TO_TICKS(duration);
}

template <typename T>
void TimeTracked<T>::receive(const T &data) {
    m_last_ticks = xTaskGetTickCount();
    m_data = data;
}

} // namespace rear
