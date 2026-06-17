#ifndef __MODULES_FLIGHT_INPUT_H

#define __MODULES_FLIGHT_INPUT_H

#include "stm32f10x.h"
#include "modules_remote.h"
#include <stdint.h>


typedef struct
{
    float roll_angle_sp;     // 横滚目标角度，单位：度
    float pitch_angle_sp;    // 俯仰目标角度，单位：度
    float yaw_rate_sp;       // 偏航目标角速度，单位：度/秒

    uint16_t throttle;       // 油门，1000~2000 或转换后的 0~1000

    uint8_t arm_request;     // 用户请求解锁
    uint8_t disarm_request;  // 用户请求上锁
} FlightCommand_t;

void FlightInput_Update(const RemoteData_t *remote, FlightCommand_t *cmd);

#endif /* __MODULES_FLIGHT_INPUT_H */