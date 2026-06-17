#ifndef __IMU_H
#define	__IMU_H


//#include "ALL_DATA.h"
#include "mpu6050.h"

//extern float GetNormAccz(void);
extern void GetAngle(const _st_Mpu *pMpu,_st_AngE *pAngE, const float dt);
extern float GetNormAccz(void);
extern void imu_rest(void);
#endif
