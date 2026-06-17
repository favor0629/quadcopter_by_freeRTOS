#include "motor.h"
// #include "mixer.h"
// #define MOTOR_MAX_NUMBER 4U



static void motor_pwm_limit(uint16_t *value)
{
    if (*value > 1000) {
        *value = 1000; // 限制最大值，防止过大导致电机损坏
    }
    if (*value < 0) {
        *value = 0; // 限制最小值，防止负数导致错误
    }
}

void motor_init(void)
{
    PWM_Init();
}

/**
 * @brief 设置电机 PWM 输出
 * @param value PWM 占空比值，范围根据 PWM 模块配置而定，通常是 0-1000 或 0-2000 等
 */
void motor_set_pwm1(uint16_t value)
{
    //motor_pwm_limit(&value);
    PWM_SetCompare1(value);
}

void motor_set_pwm2(uint16_t value)
{
    //motor_pwm_limit(&value);
    PWM_SetCompare2(value);
}

void motor_set_pwm3(uint16_t value)
{
    //motor_pwm_limit(&value);
    PWM_SetCompare3(value);
}

void motor_set_pwm4(uint16_t value)
{
    //motor_pwm_limit(&value);
    PWM_SetCompare4(value);
}

// void motor_write_all(const MotorOutput_t *motor)
// {
//     motor_set_pwm1(motor->m1);
//     motor_set_pwm2(motor->m2);
//     motor_set_pwm3(motor->m3);
//     motor_set_pwm4(motor->m4);
// }
