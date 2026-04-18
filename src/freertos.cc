#include <freertos.hh>

#include <FreeRTOS.h>

#include <cstdint>

using namespace freertos;

namespace freertos {
namespace {

Task<configMINIMAL_STACK_SIZE> s_idle_task;

} // namespace

void Mutex::init() {
    m_handle = xSemaphoreCreateMutexStatic(&m_mutex);
}

} // namespace freertos

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer,
                                   std::uint32_t *pulIdleTaskStackSize) {
    *ppxIdleTaskTCBBuffer = s_idle_task.tcb();
    *ppxIdleTaskStackBuffer = s_idle_task.stack();
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
