#ifndef __MODULES_REMOTE_H

#define __MODULES_REMOTE_H


#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"

typedef struct
{
    uint16_t roll;      // 原始遥控通道，通常 1000~2000，中位 1500
    uint16_t pitch;     
    uint16_t throttle;
    uint16_t yaw;

    uint16_t aux1;
    uint16_t aux2;
    uint16_t aux3;
    uint16_t aux4;

    uint8_t valid;      // 当前帧是否有效
    uint8_t failsafe;   // 是否进入遥控失联
} RemoteData_t;



// void Remote_Init(void);
// void Remote_Update(void);
// void Remote_GetData(RemoteData_t *out);
// uint8_t Remote_IsFailsafe(void);

/* 初始化遥控模块 */
void Remote_Init(void);

/* 遥控接收任务 */
void Remote_Task(void *argument);

/* 获取最新遥控数据快照 */
BaseType_t Remote_GetLatest(RemoteData_t *out, TickType_t timeout);

/* 获取是否失联 */
uint8_t Remote_IsFailsafe(void);


#endif /* __MODULES_REMOTE_H */