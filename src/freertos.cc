#include <freertos.hh>

#include <FreeRTOS.h>

#include <cstdint>
#include <span>

using namespace freertos;

namespace freertos {
namespace {

Task<configMINIMAL_STACK_SIZE> s_idle_task;

} // namespace

InterruptYielder::~InterruptYielder() {
    portYIELD_FROM_ISR(m_higher_priority_task_woken);
}

void Mutex::init() {
    m_handle = xSemaphoreCreateMutexStatic(&m_mutex);
}

void Mutex::lock() {
    xSemaphoreTake(m_handle, portMAX_DELAY);
}

void Mutex::unlock() {
    xSemaphoreGive(m_handle);
}

std::span<std::uint8_t> MessageBufferBase::receive(std::span<std::uint8_t> buffer, TickType_t ticks_to_wait) {
    const auto size = xMessageBufferReceive(m_handle, buffer.data(), buffer.size(), ticks_to_wait);
    return buffer.subspan(0, size);
}

std::span<std::uint8_t> MessageBufferBase::receive_isr(std::span<std::uint8_t> buffer, InterruptYielder &yielder) {
    const auto size = xMessageBufferReceiveFromISR(m_handle, buffer.data(), buffer.size(), *yielder);
    return buffer.subspan(0, size);
}

bool MessageBufferBase::send(std::span<const std::uint8_t> buffer, TickType_t ticks_to_wait) {
    return xMessageBufferSend(m_handle, buffer.data(), buffer.size(), ticks_to_wait) != 0;
}

bool MessageBufferBase::send_isr(std::span<const std::uint8_t> buffer, InterruptYielder &yielder) {
    return xMessageBufferSendFromISR(m_handle, buffer.data(), buffer.size(), *yielder) != 0;
}

PeriodScheduler::PeriodScheduler() {
    m_last_schedule_time = xTaskGetTickCount();
}

bool PeriodScheduler::delay_until_ms(std::uint32_t period_ms) {
    return xTaskDelayUntil(&m_last_schedule_time, pdMS_TO_TICKS(period_ms)) == pdTRUE;
}

} // namespace freertos

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer,
                                   std::uint32_t *pulIdleTaskStackSize) {
    *ppxIdleTaskTCBBuffer = s_idle_task.tcb();
    *ppxIdleTaskStackBuffer = s_idle_task.stack();
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
