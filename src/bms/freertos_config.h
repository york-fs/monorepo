#pragma once

// Scheduler configuration.
#define configCPU_CLOCK_HZ 56000000
#define configTICK_RATE_HZ 1000
#define configUSE_PREEMPTION 1
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0
#define configUSE_16_BIT_TICKS 0
#define configUSE_TICKLESS_IDLE 0
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configMAX_PRIORITIES 5

// Task configuration.
#define configIDLE_SHOULD_YIELD 1
#define configMINIMAL_STACK_SIZE 128

// Heap configuration.
#define configSUPPORT_DYNAMIC_ALLOCATION 1
#define configTOTAL_HEAP_SIZE (8 * 1024)
#define configUSE_MALLOC_FAILED_HOOK 0

// Interrupt configuration. The kernel gets the lowest priority, which is the highest numerical value.
#define configKERNEL_INTERRUPT_PRIORITY (15 << 4)
#define configMAX_SYSCALL_INTERRUPT_PRIORITY (5 << 4)

// Extras configuration.
#define configUSE_MUTEXES 1
#define INCLUDE_vTaskDelay 1
#define INCLUDE_xTaskDelayUntil 1
#define INCLUDE_vTaskSuspend 1
