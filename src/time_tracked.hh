#pragma once

#include <freertos.hh>

#include <atomic>
#include <cstdint>
#include <optional>

template <typename T>
class TimeTracked : public std::optional<T> {
    const std::uint32_t m_expiration_ms;
    std::atomic<TickType_t> m_last_ticks{0};

public:
    explicit TimeTracked(const std::uint32_t expiration_ms) : m_expiration_ms(expiration_ms) {}

    std::optional<T> receive(const T &data);
    void update();
};

template <typename T>
std::optional<T> TimeTracked<T>::receive(const T &data) {
    std::optional other(data);
    this->swap(other);
    m_last_ticks.store(xTaskGetTickCountFromISR());
    return other;
}

template <typename T>
void TimeTracked<T>::update() {
    if (xTaskGetTickCount() - m_last_ticks.load() >= pdMS_TO_TICKS(m_expiration_ms)) {
        this->reset();
    }
}
