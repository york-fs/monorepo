#pragma once

#include <util/type_traits.hh>

#include <FreeRTOS.h>
#include <message_buffer.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>

#include <array>
#include <cstdint>
#include <mutex>
#include <optional>
#include <span>
#include <type_traits>

namespace freertos {

class InterruptYielder {
    BaseType_t m_higher_priority_task_woken{pdFALSE};

public:
    InterruptYielder() = default;
    InterruptYielder(const InterruptYielder &) = delete;
    InterruptYielder(InterruptYielder &&) = delete;
    ~InterruptYielder();

    InterruptYielder &operator=(const InterruptYielder &) = delete;
    InterruptYielder &operator=(InterruptYielder &&) = delete;

    BaseType_t *operator*() { return &m_higher_priority_task_woken; }
};

class Mutex {
    SemaphoreHandle_t m_handle{nullptr};
    StaticSemaphore_t m_mutex;

public:
    void init();
    void lock();
    void unlock();

    template <typename Func>
    decltype(auto) with_locked(Func &&func);

    explicit operator bool() const { return m_handle != nullptr; }
    SemaphoreHandle_t operator*() const { return m_handle; }
};

class MessageBufferBase {
protected:
    MessageBufferHandle_t m_handle{nullptr};

public:
    std::span<std::uint8_t> receive(std::span<std::uint8_t> buffer, TickType_t ticks_to_wait);
    std::span<std::uint8_t> receive_isr(std::span<std::uint8_t> buffer, InterruptYielder &yielder);

    bool send(std::span<const std::uint8_t> buffer, TickType_t ticks_to_wait);
    bool send_isr(std::span<const std::uint8_t> buffer, InterruptYielder &yielder);

    explicit operator bool() const { return m_handle != nullptr; }
    MessageBufferHandle_t operator*() const { return m_handle; }
};

template <std::size_t Size>
class MessageBuffer : public MessageBufferBase {
    StaticMessageBuffer_t m_buffer;
    std::array<std::uint8_t, Size> m_storage;

public:
    void init();
};

class TaskBase {
protected:
    TaskHandle_t m_handle{nullptr};

public:
    void notify(UBaseType_t index);
    void notify_isr(UBaseType_t index, InterruptYielder &yielder);

    void notify_give(UBaseType_t index);
    void notify_give_isr(UBaseType_t index, InterruptYielder &yielder);

    void notify_set_bits(UBaseType_t index, std::uint32_t bits);
    void notify_set_bits_isr(UBaseType_t index, std::uint32_t bits, InterruptYielder &yielder);

    void notify_overwrite(UBaseType_t index, std::uint32_t value);
    void notify_overwrite_isr(UBaseType_t index, std::uint32_t value, InterruptYielder &yielder);

    explicit operator bool() const { return m_handle != nullptr; }
    TaskHandle_t operator*() const { return m_handle; }
};

template <std::uint32_t StackDepth>
class Task : public TaskBase {
    StaticTask_t m_tcb;
    std::array<StackType_t, StackDepth> m_stack;

public:
    void init(TaskFunction_t function, const char *name, UBaseType_t priority);

    StaticTask_t *tcb() { return &m_tcb; }
    StackType_t *stack() { return m_stack.data(); }
};

template <util::trivially_copyable T>
class QueueBase {
protected:
    QueueHandle_t m_handle{nullptr};

public:
    std::optional<T> receive(TickType_t ticks_to_wait);
    std::optional<T> receive_isr(InterruptYielder &yielder);

    bool send_to_front(const T &item, TickType_t ticks_to_wait);
    bool send_to_front_isr(const T &item, InterruptYielder &yielder);

    bool send_to_back(const T &item, TickType_t ticks_to_wait);
    bool send_to_back_isr(const T &item, InterruptYielder &yielder);

    void overwrite(const T &item);
    void overwrite_isr(const T &item, InterruptYielder &yielder);

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

class PeriodScheduler {
    TickType_t m_last_schedule_time;

public:
    PeriodScheduler();
    PeriodScheduler(const PeriodScheduler &) = delete;
    PeriodScheduler(PeriodScheduler &&) = delete;

    PeriodScheduler &operator=(const PeriodScheduler &) = delete;
    PeriodScheduler &operator=(PeriodScheduler &&) = delete;

    bool delay_until_ms(std::uint32_t period_ms);
};

template <typename Func>
decltype(auto) Mutex::with_locked(Func &&func) {
    std::lock_guard lock(*this);
    return func();
}

template <std::size_t Size>
void MessageBuffer<Size>::init() {
    MessageBufferBase::m_handle = xMessageBufferCreateStatic(Size, m_storage.data(), &m_buffer);
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
std::optional<T> QueueBase<T>::receive_isr(InterruptYielder &yielder) {
    T item;
    return xQueueReceiveFromISR(m_handle, &item, *yielder) == pdPASS ? std::optional(item) : std::nullopt;
}

template <util::trivially_copyable T>
bool QueueBase<T>::send_to_front(const T &item, TickType_t ticks_to_wait) {
    return xQueueSendToFront(m_handle, &item, ticks_to_wait) == pdPASS;
}

template <util::trivially_copyable T>
bool QueueBase<T>::send_to_front_isr(const T &item, InterruptYielder &yielder) {
    return xQueueSendToFrontFromISR(m_handle, &item, *yielder) == pdPASS;
}

template <util::trivially_copyable T>
bool QueueBase<T>::send_to_back(const T &item, TickType_t ticks_to_wait) {
    return xQueueSendToBack(m_handle, &item, ticks_to_wait) == pdPASS;
}

template <util::trivially_copyable T>
bool QueueBase<T>::send_to_back_isr(const T &item, InterruptYielder &yielder) {
    return xQueueSendToBackFromISR(m_handle, &item, *yielder) == pdPASS;
}

template <util::trivially_copyable T>
void QueueBase<T>::overwrite(const T &item) {
    xQueueOverwrite(m_handle, &item);
}

template <util::trivially_copyable T>
void QueueBase<T>::overwrite_isr(const T &item, InterruptYielder &yielder) {
    xQueueOverwriteFromISR(m_handle, &item, *yielder);
}

template <util::trivially_copyable T, std::uint32_t Length>
void Queue<T, Length>::init() {
    QueueBase<T>::m_handle = xQueueCreateStatic(Length, sizeof(T), m_storage.data(), &m_queue);
}

template <typename Func>
decltype(auto) in_critical_section(Func &&func) {
    taskENTER_CRITICAL();
    if constexpr (std::is_void_v<std::invoke_result_t<Func>>) {
        func();
        taskEXIT_CRITICAL();
    } else {
        auto value = func();
        taskEXIT_CRITICAL();
        return value;
    }
}

std::uint32_t notify_take(UBaseType_t index, bool clear_count_on_exit, TickType_t ticks_to_wait);
std::uint32_t notify_wait(UBaseType_t index, std::uint32_t bits_to_clear_on_entry, std::uint32_t bits_to_clear_on_exit,
                          TickType_t ticks_to_wait);

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
