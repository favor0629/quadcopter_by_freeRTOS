#ifndef __LED_H
#define __LED_H

#include "INIT.h"

#include "FreeRTOS.h"
#include "task.h"

/* ===================== LED 状态定义 ===================== */
typedef enum
{
    AlwaysOn = 1,
    AlwaysOff,
    AllFlashLight,
    AlternateFlash,
    WARNING,
    DANGEROURS,
    GET_OFFSET,
    BREATHING              /* 新增：呼吸灯模式 */
} LED_status_e;

/* ===================== LED 状态结构体 ===================== */
typedef struct
{
    uint16_t     FlashTime;   /* 闪烁时间间隔，单位 ms */
    LED_status_e status;      /* 当前 LED 状态 */
} LED_s;




/* ===================== 对外接口 ===================== */
void LED_Init(void);

/* FreeRTOS 方式初始化：初始化硬件并创建 LED 任务 */
BaseType_t LED_RTOS_Init(UBaseType_t task_priority);
void LED_Task(void *pvParameters);
/* 兼容你原来的调用习惯 */
void PilotLED(void);

/* 状态控制接口 */
uint8_t LED_GetIsInitialized(void);
void LED_SetStatus(LED_status_e status);
LED_status_e LED_GetLedStatus(void);
uint16_t LED_GetFlashTime(void);
void LED_SetFlashTime(uint16_t flash_time);
#endif