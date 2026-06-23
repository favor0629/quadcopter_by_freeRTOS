#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"


/* Idle 任务静态内存 */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleTaskStackBuffer[configMINIMAL_STACK_SIZE];

/* 如果开启了软件定时器，还需要这个 */
#if (configUSE_TIMERS == 1)
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerTaskStackBuffer[configTIMER_TASK_STACK_DEPTH];
#endif

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = xIdleTaskStackBuffer;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

#if (configUSE_TIMERS == 1)
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize )
{
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
    *ppxTimerTaskStackBuffer = xTimerTaskStackBuffer;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
#endif


#if (configCHECK_FOR_STACK_OVERFLOW > 0)
#include "debug.h"
/**
 * Stack overflow detection
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    LOG_E("[ERROR] task:%s, statck overflow!\r\n", pcTaskName);

    /* disable interrupt */
    taskDISABLE_INTERRUPTS();

    for(;;)
    {
        /* wait busy */
    }
}
#endif





