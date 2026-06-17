#include "mixer.h"
#include "myMath.h"
#include "motor.h"
#include <stdint.h>

/* ============================================================
 * 参数
 * ============================================================ */
#define MIXER_THROTTLE_BASE_MIN     1000U
#define MIXER_THROTTLE_BASE_MAX     1800U
#define MIXER_OUTPUT_MIN            0
#define MIXER_OUTPUT_MAX            1000

/* ============================================================
 * 内部辅助：清零电机输出
 * ============================================================ */
static void Mixer_ClearMotor(MotorOutput_t *motor)
{
    if (motor == NULL)
    {
        return;
    }

    motor->m1 = 0;
    motor->m2 = 0;
    motor->m3 = 0;
    motor->m4 = 0;
}

/**
 * @brief 四旋翼 X 型布局动力混控函数
 *
 * 根据油门值和飞控姿态控制输出计算四个电机的最终输出值。
 *
 * Quad-X 电机布局：
 *
 *            
 *
 *           机头      
 *   PWM2             PWM1
 *      *            *
 *        *       * 
 *    		*   *
 *      	  *  
 *    		*   *
 *        *       *
 *      *           *
 *    PWM4           PWM3
 *
 * 混控公式：
 *
 * M1 = T + Roll + Pitch + Yaw
 * M2 = T - Roll + Pitch - Yaw
 * M3 = T + Roll - Pitch - Yaw
 * M4 = T - Roll - Pitch + Yaw
 *
 * 其中：
 * T     ：基础油门
 * Roll  ：横滚控制输出
 * Pitch ：俯仰控制输出
 * Yaw   ：偏航控制输出
 *
 * 函数内部会自动对电机输出进行限幅处理。
 *
 * @param throttle 油门输入（通常范围 1000~1800）
 * @param ctrl     飞控控制输出结构体指针
 * @param state    当前飞行状态
 * @param motor    电机输出结构体指针
 *
 * @note 当 ctrl 为 NULL 或飞行状态不是 FLIGHT_FLYING 时，
 *       所有电机输出将保持为 0。
 *
 * @note 输出范围限制为 0~1000。
 */
void Mixer_QuadX(uint16_t throttle,
                 const ControlOutput_t *ctrl,
                 FlightState_t state,
                 MotorOutput_t *motor)
{
    int32_t base;
    int32_t m1, m2, m3, m4;

    if (motor == NULL)
    {
        return;
    }

    Mixer_ClearMotor(motor);

    /* 不是飞行状态，直接清零 */
    if ((ctrl == NULL) || (state != FLIGHT_FLYING))
    {
        return;
    }

    /*
     * 基础油门：
     * 1000 ~ 1800 映射成 0 ~ 800
     * 这里用 int32_t，避免中间运算溢出或类型转换不干净
     */
    base = (int32_t)throttle - (int32_t)MIXER_THROTTLE_BASE_MIN;
    base = LIMIT(base, 0, (int32_t)(MIXER_THROTTLE_BASE_MAX - MIXER_THROTTLE_BASE_MIN));

    /*
     * Quad-X 混控公式：
     * M1 = T + Roll + Pitch + Yaw
     * M2 = T - Roll + Pitch - Yaw
     * M3 = T + Roll - Pitch - Yaw
     * M4 = T - Roll - Pitch + Yaw
     */
    m1 = base + (int32_t)ctrl->roll_out + (int32_t)ctrl->pitch_out + (int32_t)ctrl->yaw_out;
    m2 = base - (int32_t)ctrl->roll_out + (int32_t)ctrl->pitch_out - (int32_t)ctrl->yaw_out;
    m3 = base + (int32_t)ctrl->roll_out - (int32_t)ctrl->pitch_out - (int32_t)ctrl->yaw_out;
    m4 = base - (int32_t)ctrl->roll_out - (int32_t)ctrl->pitch_out + (int32_t)ctrl->yaw_out;

    /*
     * 限幅到 0 ~ 1000
     * 具体上限是否为 1000，要和你的电机驱动层保持一致。
     */
    motor->m1 = (int16_t)LIMIT(m1, MIXER_OUTPUT_MIN, MIXER_OUTPUT_MAX);
    motor->m2 = (int16_t)LIMIT(m2, MIXER_OUTPUT_MIN, MIXER_OUTPUT_MAX);
    motor->m3 = (int16_t)LIMIT(m3, MIXER_OUTPUT_MIN, MIXER_OUTPUT_MAX);
    motor->m4 = (int16_t)LIMIT(m4, MIXER_OUTPUT_MIN, MIXER_OUTPUT_MAX);
}

/* ============================================================
 * 输出四路电机 PWM
 * 说明：
 * - 这里应该只在“一个任务”里调用
 * - 如果 motor 为 NULL，则直接停机
 * - 如果你的 motor_set_pwmX() 不是线程安全的，不要多任务同时调用
 * ============================================================ */
void motor_write_all(const MotorOutput_t *motor)
{
    if (motor == NULL)
    {
        motor_stop_all();
        return;
    }

    /*
     * 再做一次限幅，防止上层有异常数据。
     * 这里把负值也保护掉。
     */
    motor_set_pwm1((uint16_t)LIMIT(motor->m1, 0, MIXER_OUTPUT_MAX));
    motor_set_pwm2((uint16_t)LIMIT(motor->m2, 0, MIXER_OUTPUT_MAX));
    motor_set_pwm3((uint16_t)LIMIT(motor->m3, 0, MIXER_OUTPUT_MAX));
    motor_set_pwm4((uint16_t)LIMIT(motor->m4, 0, MIXER_OUTPUT_MAX));
}

/* ============================================================
 * 停止所有电机
 * ============================================================ */
void motor_stop_all(void)
{
    motor_set_pwm1(0U);
    motor_set_pwm2(0U);
    motor_set_pwm3(0U);
    motor_set_pwm4(0U);
}

