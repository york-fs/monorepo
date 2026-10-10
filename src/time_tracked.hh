#pragma once

#include <freertos.hh>

#include <atomic>
#include <cstdint>
#include <optional>

/**
 * @brief A class to track incoming data which goes stale and expires after a set period.
 */
template <typename T, std::uint32_t ExpirationMs>
class TimeTracked {
    std::optional<T> m_value;
    std::atomic<TickType_t> m_last_ticks{0};

    std::optional<T> value(TickType_t ticks) const;

public:
    /**
     * @brief Retrieves the stored data.
     *
     * Must not be called from an interrupt handler.
     *
     * @return T if data has been received in the last ExpirationMs period; nullopt otherwise
     */
    std::optional<T> get() const;

    /**
     * @brief Updates the stored data and refreshes the expiration timer.
     *
     * This function must be called from an interrupt handler.
     *
     * @param data the newly received data
     * @return the previously stored data if it is was received in the last ExpirationMs period; nullopt otherwise
     */
    std::optional<T> receive_isr(const T &data);
};

template <typename T, std::uint32_t ExpirationMs>
std::optional<T> TimeTracked<T, ExpirationMs>::value(TickType_t ticks) const {
    if (ticks - m_last_ticks.load() >= pdMS_TO_TICKS(ExpirationMs)) {
        return std::optional<T>();
    }
    return m_value;
}

template <typename T, std::uint32_t ExpirationMs>
std::optional<T> TimeTracked<T, ExpirationMs>::get() const {
    return freertos::in_critical_section([this] {
        return value(xTaskGetTickCount());
    });
}

template <typename T, std::uint32_t ExpirationMs>
std::optional<T> TimeTracked<T, ExpirationMs>::receive_isr(const T &data) {
    const auto old = value(xTaskGetTickCountFromISR());
    m_value.emplace(data);
    m_last_ticks.store(xTaskGetTickCountFromISR());
    return old;
}
