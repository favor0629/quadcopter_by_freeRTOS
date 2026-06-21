#ifndef __INIT_H
#define __INIT_H


#include "stm32f10x.h"
//#include "ALL_DATA.h"
//#include "sys.h"
//#include "I2C.h"
#include "SPI.h"
#include "nrf24l01.h"
#include "USART.h"
//#include  "TIM.h"
#include "led.h"
#include "mpu6050.h"
#include "imu.h"
//#include "ANO_DT.h"
//#include "reemote.h"
#include "modules_remote.h"
//#include "control.h"
#include "myMath.h"
////#include "USB_HID.h"
//#include "pwm.h"
#include "motor.h"
#include "pid.h"
#include "Delay.h"
#include "Timer.h"

// typedef struct
// {
// 	uint8_t unlock:1;
// }ALL_flag_t;

//typedef struct PidObject PidObject;

#define USE_PID_OBJECT_COUNT  6U
extern PidObject pidRateX; //内环PID数据
extern PidObject pidRateY;
extern PidObject pidRateZ;

extern PidObject pidPitch; //外环PID数据
extern PidObject pidRoll;
extern PidObject pidYaw;

uint32_t get_SysTick(void);
void ALL_flag_set(uint8_t flag);
uint8_t ALL_flag_get(void);

void ALL_Init(void);

#endif //__INIT_H
