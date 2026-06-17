#ifndef __MODULES_FLIGHT_STATUS_H
#define __MODULES_FLIGHT_STATUS_H

#include "stm32f10x.h"
#include "modules_remote.h"
#include "modules_safety.h"
#include "modules_flight_input.h"

typedef enum
{
    FLIGHT_LOCKED = 0,     // 锁定
    FLIGHT_ARMED_IDLE,     // 已解锁但油门低
    FLIGHT_FLYING,         // 正在飞行
    FLIGHT_FAILSAFE,       // 失联保护
    FLIGHT_ERROR           // 传感器或姿态异常
} FlightState_t;

/* 初始化状态机 */
void FlightState_Init(void);

/* 状态更新，只应由控制任务调用 */
void FlightState_Update(const FlightCommand_t *cmd, const SafetyStatus_t *safety);

/* 获取当前状态 */
FlightState_t FlightState_Get(void);

/* 是否允许电机输出 */
uint8_t FlightState_IsMotorEnabled(void);

#endif  /* __MODULES_FLIGHT_STATUS_H */