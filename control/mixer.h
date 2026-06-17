#ifndef __MIXER_H

#define __MIXER_H

#include "stm32f10x.h"
#include "modules_flight_status.h"
#include "flight_control.h"

/**
 * @brief Motor output values for a Quadcopter.
 *
 * This structure stores the PWM output values of the four motors.
 */
typedef struct
{
    int16_t m1;  // Motor 1 output value
    int16_t m2;  // Motor 2 output value
    int16_t m3;  // Motor 3 output value
    int16_t m4;  // Motor 4 output value
} MotorOutput_t;

void motor_write_all(const MotorOutput_t *motor);
void motor_stop_all(void);
void Mixer_QuadX(uint16_t throttle,
                 const ControlOutput_t *ctrl,
                 FlightState_t state,
                 MotorOutput_t *motor);

                 
#endif /* __MIXER_H */