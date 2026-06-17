#ifndef __MPU6050_H
#define __MPU6050_H


#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "iic.h"

typedef struct
{
    int16_t accX;
    int16_t accY;
    int16_t accZ;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
} _st_Mpu;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} _st_AngE;

typedef enum
{
    MPU6050_OK = 0,
    MPU6050_ERR,
    MPU6050_ERR_PARAM,
    MPU6050_ERR_NOT_INIT,
    MPU6050_ERR_BUSY,
    MPU6050_ERR_I2C,
    MPU6050_ERR_TIMEOUT,
    MPU6050_ERR_FLASH
} MPU6050_Status_t;

extern int16_t MpuOffset[6];

/* 初始化：建议在任务里调用，或在调度器启动前调用也可以 */
MPU6050_Status_t Mpu_Init(void);

/* 读取一次原始数据并更新内部缓存 */
MPU6050_Status_t MpuGetData(void);

/* 校准：建议在飞机静止时、控制任务暂停后调用 */
MPU6050_Status_t MpuGetOffset(void);

/* 线程安全快照接口 */
MPU6050_Status_t get_MPU6050_datas(_st_Mpu *mpu);
MPU6050_Status_t get_MPU6050_Angle(_st_AngE *pAngE);
MPU6050_Status_t set_MPU6050_Angle(const _st_AngE *pAngE);

#endif // __MPU6050_H