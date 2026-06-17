#ifndef __MODULES_SAFETY_H
#define __MODULES_SAFETY_H

#include "stm32f10x.h"
#include "modules_remote.h"
#include "mpu6050.h"
#include <stdint.h>

#define PITCH_ANGLE_MAX 50.0f
#define ROLL_ANGLE_MAX  50.0f

typedef struct
{
    uint8_t remote_lost;       /* 遥控失联 */
    uint8_t mpu_lost;          /* MPU 数据异常 */
    uint8_t angle_too_large;   /* 姿态角过大 */
    uint8_t emergency;         /* 任意紧急状态 */
} SafetyStatus_t;

/* 初始化为安全状态 */
void Safety_Init(SafetyStatus_t *safety);

/* 安全检查：输入必须是快照 */
void Safety_Update(const RemoteData_t *remote,
                   const _st_Mpu *mpu,
                   const _st_AngE *angle,
                   SafetyStatus_t *safety);

/* 是否紧急 */
uint8_t Safety_IsEmergency(const SafetyStatus_t *safety);

#endif /* __MODULES_SAFETY_H */