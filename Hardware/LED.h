#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"

void LED_Init(void);
void LED_ON(void);
void LED_OFF(void);
void LED_Turn(void);
void LED_Breath(void);
void LED_BreathEnable(uint8_t enable);
void LED_BreathTick(void);

/* FreeRTOS 下建议创建这个任务周期调用 LED_BreathTick() */
void LED_Task(void *argument);

#endif /* __LED_H */