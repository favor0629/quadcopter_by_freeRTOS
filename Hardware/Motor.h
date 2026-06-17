#ifndef __MOTOR_H

#define __MOTOR_H

#include "stm32f10x.h"
#include "pwm.h"


void motor_init(void);

void motor_set_pwm1(uint16_t value);
void motor_set_pwm2(uint16_t value);
void motor_set_pwm3(uint16_t value);
void motor_set_pwm4(uint16_t value);




#endif //__MOTOR_H
