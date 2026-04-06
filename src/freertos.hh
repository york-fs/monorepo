#pragma once

#include <util.hh>

#include <FreeRTOS.h>
#include <queue.h>

#include <cstdint>
#include <optional>

namespace freertos {

template <util::trivially_copyable T>
class Queue {
    QueueHandle_t m_handle{nullptr};

    Queue(QueueHandle_t handle) : m_handle(handle) {}

public:
    Queue() = default;

    static Queue create(std::uint32_t size);

    std::optional<T> receive(TickType_t ticks_to_wait);
    std::optional<T> receive_isr(BaseType_t *higher_priority_task_woken);

    bool send_to_front(const T &item, TickType_t ticks_to_wait);
    bool send_to_front_isr(const T &item, BaseType_t *higher_priority_task_woken);

    bool send_to_back(const T &item, TickType_t ticks_to_wait);
    bool send_to_back_isr(const T &item, BaseType_t *higher_priority_task_woken);

    void overwrite(const T &item);
    void overwrite_isr(const T &item, BaseType_t *higher_priority_task_woken);

    explicit operator bool() const { return m_handle != nullptr; }
};

template <util::trivially_copyable T>
Queue<T> Queue<T>::create(std::uint32_t size) {
    return xQueueCreate(size, sizeof(T));
}

template <util::trivially_copyable T>
std::optional<T> Queue<T>::receive(TickType_t ticks_to_wait) {
    T item;
    return xQueueReceive(m_handle, &item, ticks_to_wait) == pdPASS ? std::optional(item) : std::nullopt;
}

template <util::trivially_copyable T>
std::optional<T> Queue<T>::receive_isr(BaseType_t *higher_priority_task_woken) {
    T item;
    return xQueueReceiveFromISR(m_handle, &item, higher_priority_task_woken) == pdPASS ? std::optional(item)
                                                                                       : std::nullopt;
}

template <util::trivially_copyable T>
bool Queue<T>::send_to_front(const T &item, TickType_t ticks_to_wait) {
    return xQueueSendToFront(m_handle, &item, ticks_to_wait) == pdPASS;
}

template <util::trivially_copyable T>
bool Queue<T>::send_to_front_isr(const T &item, BaseType_t *higher_priority_task_woken) {
    return xQueueSendToFrontFromISR(m_handle, &item, higher_priority_task_woken) == pdPASS;
}

template <util::trivially_copyable T>
bool Queue<T>::send_to_back(const T &item, TickType_t ticks_to_wait) {
    return xQueueSendToBack(m_handle, &item, ticks_to_wait) == pdPASS;
}

template <util::trivially_copyable T>
bool Queue<T>::send_to_back_isr(const T &item, BaseType_t *higher_priority_task_woken) {
    return xQueueSendToBackFromISR(m_handle, &item, higher_priority_task_woken) == pdPASS;
}

template <util::trivially_copyable T>
void Queue<T>::overwrite(const T &item) {
    xQueueOverwrite(m_handle, &item);
}

template <util::trivially_copyable T>
void Queue<T>::overwrite_isr(const T &item, BaseType_t *higher_priority_task_woken) {
    xQueueOverwriteFromISR(m_handle, &item, higher_priority_task_woken);
}

/**
 * @brief Returns the scheduler uptime in milliseconds. Can be called from interrupts.
 */
inline std::uint32_t uptime_ms() {
    // This conditional should be compiled out since the tick type is atomic so no critical section or interrupt masking
    // is needed. This code also ignores the possibility of tick overflow.
    const auto tick_count = xPortIsInsideInterrupt() ? xTaskGetTickCountFromISR() : xTaskGetTickCount();
    return pdTICKS_TO_MS(tick_count);
}

} // namespace freertos
