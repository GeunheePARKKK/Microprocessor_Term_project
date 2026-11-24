// FreeRTOSConfig.h : S32K144 (Cortex-M4F, 80 MHz) for the smart parking barrier, structure (3)
//  condition variables of the experiment (plan 4-4) :
//    configTICK_RATE_HZ   1000 (1 ms tick) or 100 (10 ms tick, then main.c FND_MODE must be 1)
//    configUSE_PREEMPTION 1 (preemptive) or 0 (cooperative)
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>

#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      80000000UL
#define TP_TICK_HZ                              1000            // 1000 = 1 ms tick, 100 = 10 ms tick
#define configTICK_RATE_HZ                      ((TickType_t)TP_TICK_HZ)
#define configMAX_PRIORITIES                    7
#define configMINIMAL_STACK_SIZE                ((uint16_t)128)
#define configMAX_TASK_NAME_LEN                 8
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               0
#define configUSE_QUEUE_SETS                    0
#define configUSE_TASK_NOTIFICATIONS            1
#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configTOTAL_HEAP_SIZE                   ((size_t)(24 * 1024))
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_MALLOC_FAILED_HOOK            1
#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TRACE_FACILITY                0
#define configUSE_STATS_FORMATTING_FUNCTIONS    0
#define configUSE_CO_ROUTINES                   0
#define configUSE_TIMERS                        0
#define configENABLE_FPU                        1
#define configENABLE_MPU                        0

#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_xTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xQueueGetMutexHolder            0

// Cortex-M4 on S32K144 : 4 priority bits
#define configPRIO_BITS                         4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5
#define configKERNEL_INTERRUPT_PRIORITY         (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

#define configASSERT(x)  if ((x) == 0) { taskDISABLE_INTERRUPTS(); for (;;) {} }

// map the port handlers onto the names in startup_S32K144.S
// SysTick_Handler is written in main.c (it wraps xPortSysTickHandler to measure M2)
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler

// count context switches for M3 (defined in main.c)
extern volatile uint32_t ctx_switch_count;
#define traceTASK_SWITCHED_IN()   (ctx_switch_count++)

#endif
