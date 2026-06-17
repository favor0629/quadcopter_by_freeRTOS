#include "modules_safety.h"
#include <string.h>

static void Safety_SetDefault(SafetyStatus_t *safety)
{
    if(safety == NULL)
    {
        return;
    }
    safety->remote_lost = 1U;
    safety->mpu_lost = 1U;
    safety->angle_too_large = 1U;

    safety->emergency = 1U;
}


void Safety_Init(SafetyStatus_t *safety)
{
    if(safety == NULL)
    {
        return;
    }

    Safety_SetDefault(safety);
}


void Safety_Update(const RemoteData_t *remote, const _st_Mpu *mpu, const _st_AngE *angle, SafetyStatus_t *safety)
{
    uint8_t remote_lost;
    uint8_t mpu_lost;
    uint8_t angle_too_large;

    if(safety == NULL)
    {
        return;
    }

    /* 先全部默认成为安全失败状态 */
    Safety_SetDefault(safety);

    /* 遥控失联判定 */
    if(remote == NULL)
    {
        remote_lost = 1U;
    }
    else
    {
        remote_lost = (remote->failsafe != 0U) ? 1U : 0U;
    }

    /* MPU 失效判定
       这里你当前结构体里没有给出明确的“在线/失效”字段，
       所以先按“指针为空就认为失效”处理。
       如果你的 _st_Mpu 里有有效标志，后面应该换成真实字段。
    */
   if(mpu == NULL)
   {
        mpu_lost = 1U;
   }
   else
   {
        mpu_lost = 0U;
   }

   /* 姿态角过大判定 */
   if(angle == NULL)
   {
        angle_too_large = 1U;
   }
   else
   {
        angle_too_large = 
            ((angle->roll > ROLL_ANGLE_MAX) ||
            (angle->roll < -ROLL_ANGLE_MAX) ||
            (angle->pitch > PITCH_ANGLE_MAX) ||
            (angle->pitch < -PITCH_ANGLE_MAX)) ? 1U : 0U;
   }

   /* 输出结果 */
   safety->remote_lost = remote_lost;
   safety->mpu_lost = mpu_lost;
   safety->angle_too_large = angle_too_large;
   safety->emergency = ( remote_lost || mpu_lost || angle_too_large) ? 1U : 0U;
}

uint8_t Safety_IsEmergency(const SafetyStatus_t *safety)
{
    if(safety == NULL)
    {
        return 1U;
    }
    return (safety->emergency != 0U) ? 1U : 0U;
}

// #include "modules_safety.h"

// void Safety_Update(const RemoteData_t *remote,
//                    const _st_Mpu *mpu,
//                    const _st_AngE *angle,
//                    SafetyStatus_t *safety)
// {
//     (void)mpu;

//     if(safety == 0)
//     {
//         return;
//     }

//     safety->remote_lost = 1;
//     safety->mpu_lost = 0;
//     safety->angle_too_large = 1;
//     safety->emergency = 1;

//     if(remote == 0 || angle == 0)
//     {
//         return;
//     }

//     safety->remote_lost = remote->failsafe;
//     safety->angle_too_large = (angle->roll > ROLL_ANGLE_MAX ||
//                                angle->roll < -ROLL_ANGLE_MAX ||
//                                angle->pitch > PITCH_ANGLE_MAX ||
//                                angle->pitch < -PITCH_ANGLE_MAX) ? 1U : 0U;
//     safety->mpu_lost = 0;
//     safety->emergency = (safety->remote_lost || safety->mpu_lost || safety->angle_too_large) ? 1U : 0U;
// }
