/**
 * @file    flight_control.c
 * @brief   Flight attitude controller implementation.
 *
 * @details
 * This module implements the flight attitude control loop.
 *
 * Control architecture:
 *
 *     Desired Angle
 *           │
 *           ▼
 *      Angle PID
 *           │
 *           ▼
 *    Desired Rate
 *           │
 *           ▼
 *      Rate PID
 *           │
 *           ▼
 *     Motor Mixer
 *
 * The controller uses a cascade PID structure:
 * - Outer loop: attitude control
 * - Inner loop: angular rate control
 *
 * Supported axes:
 * - Roll
 * - Pitch
 * - Yaw
 */

#include "flight_control.h"
#include "INIT.h"

/*==============================================================================
 * Global variable 
 *============================================================================*/
/**
 * @brief PID object table.
 *
 * @details
 * Stores pointers to all PID controllers used by the
 * flight control module.
 *
 * This table allows batch operations such as:
 * - PID reset
 * - PID initialization
 * - Parameter update
 */
static PidObject *flight_pid_objects[] = {
    &pidRateX,
    &pidRateY,
    &pidRateZ,
    &pidRoll,
    &pidPitch,
    &pidYaw
};

#define FLIGHT_PID_COUNT ((uint8_t)(sizeof(flight_pid_objects)) / (sizeof(flight_pid_objects[0])))


/* 上一次飞行状态，用于检测状态切换 */
static FlightState_t s_last_state = FLIGHT_LOCKED;

/* 偏航目标角， 避免直接在pidYaw.desired上直接累加导致状态切换跳变*/
static float s_yaw_target = 0.0f;

/* 初始化标志 */
static uint8_t s_inited = 0U;

/* yaw 控制限幅，防止偏航通道输出过大导致电机剧烈反转 */
#define FLIGHT_YAW_TARGET_LIMIT_DEG      180.0f
#define FLIGHT_YAW_OUTER_LIMIT_DEG_S     120.0f
#define FLIGHT_YAW_INNER_LIMIT           250.0f

#define YAW_MAX_RATE_DEF_S              10.0f  // yaw 每秒最多变化30度
#define YAW_INPUT_DEADBAND              0.5f       // 摇杆死区
#define YAW_INPUT_EXPO                  0.3f    // 0-1,越小越钝

/* ============================================================
 * 内部辅助：安全输出清零
 * ============================================================ */
static void FlightControl_ClearOutput(ControlOutput_t *out)
{
    if(out == NULL)
    {
        return;
    }

    out->pitch_out = 0.0f;
    out->roll_out = 0.0f;
    out->yaw_out = 0.0f;
}

static float apply_deadband(const float x, const float deadband)
{
    if(x > -deadband && x < deadband)
    {
        return 0.0f;
    }
    return x;
}

static float apply_expo(const float x, float expo)
{
    // 简单 expo：低速更柔和，高速保留
    return expo * x * x * x + (1.0f - expo) * x;
}


/* ============================================================
 * 初始化
 * ============================================================ */
void FlightControl_Init(void)
{
    if(s_inited != 0U)
    {
        return;
    }
    s_last_state = FLIGHT_LOCKED;
    s_inited = 1U;
    s_yaw_target = 0.0f;

    pidRest(flight_pid_objects, FLIGHT_PID_COUNT);

    pidRoll.desired = 0.0f;
    pidPitch.desired = 0.0f;
    pidYaw.desired = 0.0f;

    pidRateX.desired = 0.0f;
    pidRateY.desired = 0.0f;
    pidRateZ.desired = 0.0f;
}


/* ============================================================
 * 重置控制器
 * 适用场景：
 * - 上锁
 * - 失联
 * - 传感器异常
 * - dt 异常
 * ============================================================ */
void FlightControl_Reset(void)
{
    pidRest(flight_pid_objects, FLIGHT_PID_COUNT);

    pidRoll.desired = 0.0f;
    pidPitch.desired = 0.0f;
    pidYaw.desired = 0.0f;

    pidRateX.desired = 0.0f;
    pidRateY.desired = 0.0f;
    pidRateZ.desired = 0.0f;

    s_yaw_target = 0.0f;
}

/* ============================================================
 * 内部辅助：进入飞行状态时初始化 yaw 目标
 * ============================================================ */
static void FlightControl_OnEnterFlying(const _st_AngE *angle)
{
    if (angle != NULL)
    {
        s_yaw_target = angle->yaw;
    }
    else
    {
        s_yaw_target = 0.0f;
    }

    pidYaw.desired = s_yaw_target;
}
/* ============================================================
 * 单步控制更新
 * ============================================================ */
void FlightControl_Update(const _st_Mpu *mpu,
                          const _st_AngE *angle,
                          const FlightCommand_t *cmd,
                          FlightState_t state,
                          float dt,
                          ControlOutput_t *out)
{
    if(out == NULL)
    {
        return;
    }

    /* 清除输出 */
    FlightControl_ClearOutput(out);

    /* 初始化保护 */
    if(s_inited != 1U)
    {
        FlightControl_Init();
    }

    /* 参数合法性检查 */
    if(mpu == NULL || angle == NULL || cmd == NULL || dt <= 0.0f)
    {
        FlightControl_Reset();
        s_last_state = state;
        return;
    }

    /* 非飞行状态，直接复位控制器，输出保持0 */
    if(state != FLIGHT_FLYING)
    {
        /* 状态切换时，复位控制器，避免积分残留 */
        if(s_last_state == FLIGHT_FLYING)
        {
            FlightControl_Reset();
        }

        s_last_state = state;
        return;
    }

    /* 切换到飞行状态，先初始化yaw目标 */
    if(s_last_state != FLIGHT_FLYING)
    {
        FlightControl_Reset();
        FlightControl_OnEnterFlying(angle);
    }

    /*
     * 设定值更新
     * 外环：角度环
     * 内环：角速度环
     */
    pidRoll.desired = cmd->roll_angle_sp;
    pidPitch.desired = cmd->pitch_angle_sp;

    /*
     * yaw 使用“角速度积分成角度目标”的方式
     * 注意：这里必须保证 dt 稳定
     */
    // s_yaw_target += cmd->yaw_rate_sp * dt;
    // s_yaw_target = LIMIT(s_yaw_target,
    //                      angle->yaw - FLIGHT_YAW_TARGET_LIMIT_DEG,
    //                      angle->yaw + FLIGHT_YAW_TARGET_LIMIT_DEG);
    // pidYaw.desired = s_yaw_target;
    float yaw_input = cmd->yaw_rate_sp;

    /* 死区限制 */
    yaw_input = apply_deadband(yaw_input, YAW_INPUT_DEADBAND);

    /*曲线压缩，减弱中间段灵敏度 */
    yaw_input = apply_expo(yaw_input, YAW_INPUT_EXPO);

    /* 映射成实际角度 */
    yaw_input *= YAW_MAX_RATE_DEF_S;

    s_yaw_target += yaw_input * dt;

    pidYaw.desired = s_yaw_target;

    /* 角速度测量值，单位需与你的 PID 设计一致 */
    pidRateX.measured = mpu->gyroX * Gyro_G;
    pidRateY.measured = mpu->gyroY * Gyro_G;
    pidRateZ.measured = mpu->gyroZ * Gyro_G;

    /* 姿态角测量值 */
    pidRoll.measured = angle->roll;
    pidPitch.measured = angle->pitch;
    pidYaw.measured = angle->yaw;

    /*
     * 如果你希望在电机未解锁/油门过低时抑制某些环路，
     * 可以在这里增加进一步的限制。
     * 目前保持你的原有 cascade 结构。
     */
    CascadePID(&pidRateX, &pidRoll, dt);
    CascadePID(&pidRateY, &pidPitch, dt);
    CascadePID(&pidRateZ, &pidYaw, dt);

    /*
     * yaw 单独限幅：先限制外环输出，再进入内环，避免积分把输出放大到很大。
     */
    // pidUpdate(&pidYaw, dt);
    // pidYaw.out = LIMIT(pidYaw.out,
    //                    -FLIGHT_YAW_OUTER_LIMIT_DEG_S,
    //                    FLIGHT_YAW_OUTER_LIMIT_DEG_S);
    // pidRateZ.desired = pidYaw.out;
    // pidUpdate(&pidRateZ, dt);

    out->roll_out = pidRateX.out;
    out->pitch_out = pidRateY.out;
    out->yaw_out = LIMIT(pidRateZ.out,
                         -FLIGHT_YAW_INNER_LIMIT,
                         FLIGHT_YAW_INNER_LIMIT);

    s_last_state = state;

}

