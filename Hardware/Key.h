#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "stm32f10x_conf.h"
#include "task.h"

extern volatile uint8_t g_key_number;

void Key_Init(void);
uint8_t Key_GetNum(void);
void Key_Tick(void);

/* FreeRTOS 下用于周期扫描按键的任务函数 */
void Key_Task(void *argument);

#endif


// #ifndef __KEY_H_
// #define __KEY_H_

// #include "FreeRTOS.h"
// #include "task.h"

// void Key_Init(void);
// void Key_SetTask1Handle(TaskHandle_t handle);
// uint8_t Key_GetNum(void);

// #endif