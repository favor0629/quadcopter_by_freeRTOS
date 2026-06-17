#include "modules_flight_status.h"
#include "imu.h"

#ifdef DEBUG
#include "debug.h"
#endif

#include "FreeRTOS.h"
#include "task.h"

static volatile FlightState_t s_current_flight_state = FLIGHT_LOCKED;

/* ------------------------------------------------------------
 * 内部函数：状态赋值
 * ------------------------------------------------------------ */
static void FlightState_Set(FlightState_t new_state)
{
    taskENTER_CRITICAL();
    s_current_flight_state = new_state;
    taskEXIT_CRITICAL();
}

/* ------------------------------------------------------------
 * 初始化
 * ------------------------------------------------------------ */
void FlightState_Init(void)
{
    taskENTER_CRITICAL();
    s_current_flight_state = FLIGHT_LOCKED;
    taskEXIT_CRITICAL();
}

/* ------------------------------------------------------------
 * 状态更新
 * 说明：
 * 1. 只能由一个控制任务调用
 * 2. cmd / safety 必须是快照，不要是正在被别的任务修改的共享对象
 * ------------------------------------------------------------ */
void FlightState_Update(const FlightCommand_t *cmd, const SafetyStatus_t *safety)
{
    FlightState_t current_state;

    if(cmd == NULL || safety == NULL)
    {
        current_state = FLIGHT_ERROR;
        FlightState_Set(current_state);
        return;
    }

    current_state = s_current_flight_state;

    /* 安全检查 */
    if(safety->angle_too_large || safety->mpu_lost)
    {
        current_state = FLIGHT_ERROR;
        FlightState_Set(current_state);
        return;
    }

    if(safety->remote_lost)
    {
        current_state = FLIGHT_FAILSAFE;
        FlightState_Set(current_state);
        return;
    }

    /* 用户强制上锁优先级高于自动状态机 */
    if(cmd->disarm_request)
    {
        current_state = FLIGHT_LOCKED;
        FlightState_Set(current_state);
        return;
    }

    switch(current_state)
    {
        case FLIGHT_LOCKED:
        {
            if(cmd->arm_request)
            {
                current_state = FLIGHT_ARMED_IDLE;
            }
            break;
        }

        case FLIGHT_ARMED_IDLE:
        {
            if(cmd->throttle > 1100U)
            {
                /*
                 * 注意：
                 * 如果 imu_rest() 内部会阻塞很久，
                 * 更推荐改成“设置标志，由 IMU 任务去执行”。
                 */
                imu_rest();
                current_state = FLIGHT_FLYING;
            }
            break;
        }

        case FLIGHT_FLYING:
        {
            if(cmd->throttle < 1020U)
            {
                current_state = FLIGHT_ARMED_IDLE;
            }
            break;
        }

        case FLIGHT_FAILSAFE:
        case FLIGHT_ERROR:
        {
            current_state = FLIGHT_LOCKED;
            break;
        }

        default:
            current_state = FLIGHT_LOCKED;
            break;
    }

    /* 更新状态 */
    FlightState_Set(current_state);

#ifdef DEBUG
    LOG_W("current_flight_state:%d\r\n", current_flight_state);
#endif
    //return;
}

/* ------------------------------------------------------------
 * 获取当前状态
 * ------------------------------------------------------------ */
FlightState_t FlightState_Get(void)
{
    FlightState_t state;

    taskENTER_CRITICAL();
    state = s_current_flight_state;
    taskEXIT_CRITICAL();
    
    return state;
}
/* ------------------------------------------------------------
 * 是否允许电机输出
 * ------------------------------------------------------------ */
uint8_t FlightState_IsMotorEnabled(void)
{
    return ((FlightState_Get() == FLIGHT_FLYING)? 1U : 0U);
}

// static FlightState_t current_flight_state = FLIGHT_LOCKED;      // 初始状态设置为上锁状态

// /**
//  * @brief 飞行状态的更新，根据状态机进行更新
//  * @param cmd 输入参数，根据FlightCommand_t结构体中的数据进行判断
//  * @param safety 输入参数，进行安全性的判断
//  */
// void FlightState_Update(const FlightCommand_t *cmd, const SafetyStatus_t *safety)
// {
//     /*安全检测*/
//     if(cmd == 0 || safety == 0)
//     {
//         current_flight_state = FLIGHT_ERROR;
//         return;
//     }

//     /*判断角度是否过大，mpu是否正常，但是在此项目中未对mup进行检测*/
//     if(safety->angle_too_large || safety->mpu_lost)
//     {
//         current_flight_state = FLIGHT_ERROR;
//         return;
//     }

//     /*判断remote摇杆数据是否有效*/
//     if(safety->remote_lost)
//     {
//         current_flight_state = FLIGHT_FAILSAFE;
//         return;
//     }

//     /*用户上锁判断*/
//     if(cmd->disarm_request)
//     {
//         current_flight_state = FLIGHT_LOCKED;
//         return;
//     }
    
//     #ifdef DEBUG
//     LOG_W("++++++++++++++now start current flight state change+++++++++++++++++++\r\n");
//     #endif

//     /*进行状态机的更新*/
//     switch(current_flight_state)
//     {
//         case FLIGHT_LOCKED:
//             /*上锁状态，如果此时收到用户解锁的命令，则进入解锁状态*/
//             if(cmd->arm_request)
//             {
                
//                 current_flight_state = FLIGHT_ARMED_IDLE;
                
//             }
//             break;

//         case FLIGHT_ARMED_IDLE:
//             /*解锁但油门低状态，如果油门拉高，则进入飞行状态*/
//             if(cmd->throttle > 1100)
//             {
//                 imu_rest();
//                 current_flight_state = FLIGHT_FLYING;
//             }
//             break;

//         case FLIGHT_FLYING:
//             /*飞行状态，如果油门拉到最低，则进入FLIGHT_ARMED_IDLE状态*/

//             if(cmd->throttle < 1020)
//             {
//                 current_flight_state = FLIGHT_ARMED_IDLE;
//             }
//             break;

//         case FLIGHT_FAILSAFE:
//         case FLIGHT_ERROR:
//             current_flight_state = FLIGHT_LOCKED;
//             break;

//         default:
//             current_flight_state = FLIGHT_LOCKED;
//             break;
//     }

// 	#ifdef DEBUG
//     LOG_W("current_flight_state:%d\r\n", current_flight_state);
// 	#endif
// }

// /**
//  * @brief 获取当前飞行状态
//  * @return 当前飞行状态，类型为 FlightState_t
//  */
// FlightState_t FlightState_Get(void)
// {
//     return current_flight_state;
// }

// /**
//  * @brief 检查当前飞行状态是否允许电机输出
//  * @return 1：如果当前状态允许电机输出（即正在飞行），0：反之
//  */
// uint8_t FlightState_IsMotorEnabled(void)
// {
//     return (current_flight_state == FLIGHT_FLYING) ? 1U : 0U;
// }
