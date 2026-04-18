#pragma once

#include <util.hh>

#include <FreeRTOS.h>
#include <message_buffer.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>

#include <array>
#include <cstdint>
#include <optional>

namespace freertos {

class Mutex {
    SemaphoreHandle_t m_handle{nullptr};
    StaticSemaphore_t m_mutex;

public:
    void init();

    explicit operator bool() const { return m_handle != nullptr; }
    SemaphoreHandle_t operator*() const { return m_handle; }
};

template <std::size_t Size>
class MessageBuffer {
    MessageBufferHandle_t m_handle{nullptr};
    StaticMessageBuffer_t m_buffer;
    std::array<std::uint8_t, Size> m_storage;

public:
    void init();

    explicit operator bool() const { return m_handle != nullptr; }
    MessageBufferHandle_t operator*() const { return m_handle; }
};

template <std::uint32_t StackDepth>
class Task {
    TaskHandle_t m_handle{nullptr};
    StaticTask_t m_tcb;
    std::array<StackType_t, StackDepth> m_stack;

public:
    void init(TaskFunction_t function, const char *name, UBaseType_t priority);

    explicit operator bool() const { return m_handle != nullptr; }
    TaskHandle_t operator*() const { return m_handle; }
    StaticTask_t *tcb() { return &m_tcb; }
    StackType_t *stack() { return m_stack.data(); }
};

template <util::trivially_copyable T>
class QueueBase {
protected:
    QueueHandle_t m_handle{nullptr};

public:
    std::optional<T> receive(TickType_t ticks_to_wait);
    std::optional<T> receive_isr(BaseType_t *higher_priority_task_woken);

    bool send_to_front(const T &item, TickType_t ticks_to_wait);
    bool send_to_front_isr(const T &item, BaseType_t *higher_priority_task_woken);

    bool send_to_back(const T &item, TickType_t ticks_to_wait);
    bool send_to_back_isr(const T &item, BaseType_t *higher_priority_task_woken);

    void overwrite(const T &item);
    void overwrite_isr(const T &item, BaseType_t *higher_priority_task_woken);

    explicit operator bool() const { return m_handle != nullptr; }
    QueueHandle_t operator*() const { return m_handle; }
};

template <util::trivially_copyable T, std::uint32_t Length>
class Queue : public QueueBase<T> {
    StaticQueue_t m_queue;
    std::array<std::uint8_t, Length * sizeof(T)> m_storage;

public:
    void init();
};

template <std::size_t Size>
void MessageBuffer<Size>::init() {
    m_handle = xMessageBufferCreateStatic(Size, m_storage.data(), &m_buffer);
}

template <std::uint32_t StackDepth>
void Task<StackDepth>::init(TaskFunction_t function, const char *name, UBaseType_t priority) {
    m_handle = xTaskCreateStatic(function, name, StackDepth, nullptr, priority, m_stack.data(), &m_tcb);
}

template <util::trivially_copyable T>
std::optional<T> QueueBase<T>::receive(TickType_t ticks_to_wait) {
    T item;
    return xQueueReceive(m_handle, &item, ticks_to_wait) == pdPASS ? std::optional(item) : std::nullopt;
}

template <util::trivially_copyable T>
std::optional<T> QueueBase<T>::receive_isr(BaseType_t *higher_priority_task_woken) {
    T item;
    return xQueueReceiveFromISR(m_handle, &item, higher_priority_task_woken) == pdPASS ? std::optional(item)
                                                                                       : std::nullopt;
}

template <util::trivially_copyable T>
bool QueueBase<T>::send_to_front(const T &item, TickType_t ticks_to_wait) {
    return xQueueSendToFront(m_handle, &item, ticks_to_wait) == pdPASS;
}

template <util::trivially_copyable T>
bool QueueBase<T>::send_to_front_isr(const T &item, BaseType_t *higher_priority_task_woken) {
    return xQueueSendToFrontFromISR(m_handle, &item, higher_priority_task_woken) == pdPASS;
}

template <util::trivially_copyable T>
bool QueueBase<T>::send_to_back(const T &item, TickType_t ticks_to_wait) {
    return xQueueSendToBack(m_handle, &item, ticks_to_wait) == pdPASS;
}

template <util::trivially_copyable T>
bool QueueBase<T>::send_to_back_isr(const T &item, BaseType_t *higher_priority_task_woken) {
    return xQueueSendToBackFromISR(m_handle, &item, higher_priority_task_woken) == pdPASS;
}

template <util::trivially_copyable T>
void QueueBase<T>::overwrite(const T &item) {
    xQueueOverwrite(m_handle, &item);
}

template <util::trivially_copyable T>
void QueueBase<T>::overwrite_isr(const T &item, BaseType_t *higher_priority_task_woken) {
    xQueueOverwriteFromISR(m_handle, &item, higher_priority_task_woken);
}

template <util::trivially_copyable T, std::uint32_t Length>
void Queue<T, Length>::init() {
    QueueBase<T>::m_handle = xQueueCreateStatic(Length, sizeof(T), m_storage.data(), &m_queue);
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
