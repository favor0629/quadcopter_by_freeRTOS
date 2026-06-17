#include "modules_flight_input.h"
#include "FreeRTOS.h"
#include "task.h"
#include "myMath.h"
//#include "debug.h"


#define FLIGHT_INPUT_ANGLE_SCALE     0.04f          // 遥控输入到目标角度的比例，单位：度/遥控单位
#define FLIGHT_INPUT_YAW_RATE_SCALE  0.5f           // 遥控输入到偏航角速度的比例，单位：度/秒/遥控单位


#define THROTTLE_MIN                 1000U

/**
 * @brief 将遥控输入转换为飞行控制命令
 * @param remote 输入参数，指向 RemoteData_t 结构体，包含遥控器数据
 * @param cmd 输出参数，指向 FlightCommand_t 结构体，包含飞行控制命令
 * @note 1. 如果 remote 无效或进入失联状态，cmd 将被设置为安全状态（油门最低，角度目标为 0，解锁请求为 0，上锁请求为 1）
 *       2. 如果 remote 有效且正常，cmd 将根据遥控输入计算目标角度、偏航角速度和油门，并根据特定的摇杆组合设置解锁/上锁请求
 */
void FlightInput_Update(const RemoteData_t *remote, FlightCommand_t *cmd)
{
    if(cmd == NULL)
    {
        return;
    }

    cmd->roll_angle_sp = 0.0f;
    cmd->pitch_angle_sp = 0.0f;
    cmd->yaw_rate_sp = 0.0f;
    cmd->throttle = THROTTLE_MIN;
    cmd->arm_request = 0;
    cmd->disarm_request = 1;   // 这里暂且设置为上锁状态，需要后续对是否上锁/解锁进行一个判断


    if(remote == NULL)
    {
        return ;
    }

    // 这里使用remote数据的快照，防止每次得到的remote可能不是同一帧
    RemoteData_t remote_snapshot;
    taskENTER_CRITICAL();
    remote_snapshot = *remote;
    taskEXIT_CRITICAL();

    if(remote_snapshot.failsafe || !remote_snapshot.valid)
    {
        return;
    }

    cmd->roll_angle_sp = -((float)remote_snapshot.roll - 1500.0f) * FLIGHT_INPUT_ANGLE_SCALE;
    cmd->pitch_angle_sp = -((float)remote_snapshot.pitch - 1500.0f) * FLIGHT_INPUT_ANGLE_SCALE;

    if(remote_snapshot.yaw > 1520 || remote_snapshot.yaw < 1480)
    {
        cmd->yaw_rate_sp = -((float)remote_snapshot.yaw - 1500.0f) * FLIGHT_INPUT_YAW_RATE_SCALE;
    }

    cmd->throttle = LIMIT(remote_snapshot.throttle, 1000, 2000);

    // 这里需要优化，或者这里并符合要求
    //cmd->arm_request = (remote->throttle < 1100 && remote->yaw > 1850) ? 1U : 0U;


    /*油门遥杆左下角锁定*/
    //cmd->disarm_request = (remote->throttle < 1050 && remote->yaw < 1150) ? 1U : 0U;

    //Debug_Printf("++++++++++++++++++++++++++++++FlightInput_Update function++++++++++++++++++++++++++\r\n");
    
}
