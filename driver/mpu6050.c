#include "mpu6050.h"
#include "filter.h"
#include "myMath.h"
#include <string.h>
#include "kalman.h"
#include "flash.h"
#include "LED.h"
#include "Delay.h"

/* ============================================================
 * MPU6050 寄存器定义
 * ============================================================ */
#define SMPLRT_DIV          0x19
#define CONFIGL             0x1A
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C

#define ACCEL_XOUT_H        0x3B
#define GYRO_XOUT_H         0x43
#define PWR_MGMT_1          0x6B
#define WHO_AM_I            0x75

#define MPU6050_PRODUCT_ID  0x68
#define MPU6050_ADDRESS     0x68

/* ============================================================
 * 参数配置
 * ============================================================ */
#define MPU_I2C_TIMEOUT_MS          20U
#define MPU_INIT_RETRY_COUNT        5U
#define MPU_INIT_DELAY_MS           30U

#define MPU_CALIB_STABLE_STEP       5
#define MPU_CALIB_STABLE_MIN       (-5)
#define MPU_CALIB_DISCARD_COUNT     100U
#define MPU_CALIB_COLLECT_COUNT     256U
#define MPU_CALIB_MAX_WAIT_MS       5000U
#define MPU_CALIB_SAMPLE_DELAY_MS   10U

#define MPU_GYRO_LPF_FACTOR         0.25f


/* ============================================================
 * 模块内状态
 * ============================================================ */
static _st_Mpu s_mpu_raw = {0};
static _st_AngE s_angle = {0};

int16_t MpuOffset[6] = {0};

static Kalman1D_t s_ekf[3] =
{
    {.x = 8192.0f, .P = 0.02f, .Q = 0.001f, .R = 0.543f, .initialized = 1},
    {.x = 8192.0f, .P = 0.02f, .Q = 0.001f, .R = 0.543f, .initialized = 1},
    {.x = 8192.0f, .P = 0.02f, .Q = 0.001f, .R = 0.543f, .initialized = 1}
};


static float s_gyro_lpf[3] = {0.0f, 0.0f, 0.0f};

static StaticSemaphore_t s_mpu_mutex_buf;
static SemaphoreHandle_t s_mpu_mutex = NULL;

static uint8_t s_mpu_inited = 0U;

/* ============================================================
 * 工具函数
 * ============================================================ */
static void MPU6050_DelayMs(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
    // #if(INCLUDE_vTaskDelay == 1)
    //     if(xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    //     {
    //         vTaskDelay(pdMS_TO_TICKS(ms));
    //     }
    // #else
    //     Delay_ms(ms);
    // #endif
}


static MPU6050_Status_t MPU6050_Lock(TickType_t wait)
{
    if(s_mpu_mutex == NULL || s_mpu_inited == 0U)
    {
        return MPU6050_ERR_NOT_INIT;
    }

    if(xSemaphoreTake(s_mpu_mutex, wait) != pdTRUE)
    {
        return MPU6050_ERR_TIMEOUT;
    }

    return MPU6050_OK;
}

static MPU6050_Status_t MPU6050_Unlock(void)
{
    if(s_mpu_mutex == NULL || s_mpu_inited == 0U)
    {
        return MPU6050_ERR_NOT_INIT;
    }

    if(xSemaphoreGive(s_mpu_mutex) != pdTRUE)
    {
        return MPU6050_ERR;
    }

    return MPU6050_OK;
}


static MPU6050_Status_t MPU6050_WriteReg(uint8_t reg, uint8_t value)
{
    if(IIC_Write_One_Byte(MPU6050_ADDRESS, reg, value) != IIC_OK)
    {
        return MPU6050_ERR_I2C;
    }
    return MPU6050_OK;
}

static MPU6050_Status_t MPU6050_ReadReg(uint8_t reg, uint8_t *value)
{
    if(value == NULL)
    {
        return MPU6050_ERR_PARAM;
    }
    IIC_Status_t ret;
    ret = IIC_Read_One_Byte(MPU6050_ADDRESS, reg, value);
    if(ret != IIC_OK)
    {
        return MPU6050_ERR_I2C;
    }

    return MPU6050_OK;
}

/* 读 12 字节原始数据，不做偏移修正，不做滤波 */
static MPU6050_Status_t MPU6050_ReadRawBlock(int16_t *out)
{
    uint8_t buffer[12];

    if(out == NULL)
    {
        return MPU6050_ERR_PARAM;
    }

    if(IIC_Read_Bytes(MPU6050_ADDRESS, ACCEL_XOUT_H, buffer, 6) != IIC_OK)
    {
        return MPU6050_ERR_I2C;
    }

    if(IIC_read_Bytes(MPU6050_ADDRESS, GYRO_XOUT_H, buffer + 6, 6) != IIC_OK)
    {
        return MPU6050_ERR_I2C;
    }

    // 读取数据成功
    out[0] = (int16_t)((((int16_t)buffer[0] << 8) | buffer[1]));
    out[1] = (int16_t)((((int16_t)buffer[2] << 8) | buffer[3]));
    out[2] = (int16_t)((((int16_t)buffer[4] << 8) | buffer[5]));
    out[3] = (int16_t)((((int16_t)buffer[6] << 8) | buffer[7]));
    out[4] = (int16_t)((((int16_t)buffer[8] << 8) | buffer[9]));
    out[5] = (int16_t)((((int16_t)buffer[10] << 8) | buffer[11]));

    return MPU6050_OK;

}


/* ============================================================
 * 初始化
 * ============================================================ */
MPU6050_Status_t Mpu_Init(void)
{
    MPU6050_Status_t ret;
    uint8_t who_am_i = 0;
    uint8_t retry;

    if(s_mpu_mutex == NULL || s_mpu_inited != 0U)
    {

#if (configSUPPORT_STATIC_ALLOCATION == 1)
        s_mpu_mutex = xSemaphoreCreateMutexStatic(&s_mpu_mutex_buf);
#endif
        if(s_mpu_mutex == NULL)
        {
            return MPU6050_ERR_NOT_INIT;
        }
    }

    ret = MPU6050_Lock(pdMS_TO_TICKS(MPU_I2C_TIMEOUT_MS));
    if(ret != MPU6050_OK)
    {
        return ret;
    }

    s_mpu_inited = 0U;

    for(retry = 0; retry < MPU_INIT_RETRY_COUNT; ++retry)
    {
        ret = MPU6050_WriteReg(PWR_MGMT_1, 0x80);  // 复位
        if(ret == MPU6050_OK)
        {
            MPU6050_DelayMs(MPU_INIT_DELAY_MS);

            ret = MPU6050_WriteReg(SMPLRT_DIV, 0x02);
            if (ret != MPU6050_OK) break;

            ret = MPU6050_WriteReg(PWR_MGMT_1, 0x01); /* 时钟源：陀螺仪Z轴 */
            if (ret != MPU6050_OK) break;

            ret = MPU6050_WriteReg(CONFIGL, 0x03);    /* DLPF */
            if (ret != MPU6050_OK) break;

            ret = MPU6050_WriteReg(GYRO_CONFIG, 0x18); /* ±2000 dps */
            if (ret != MPU6050_OK) break;

            ret = MPU6050_WriteReg(ACCEL_CONFIG, 0x09); /* ±4g */
            if (ret != MPU6050_OK) break;

            ret = MPU6050_ReadReg(WHO_AM_I, &who_am_i);
            if (ret != MPU6050_OK) break;

            if(who_am_i == MPU6050_PRODUCT_ID)  // 成功验证
            {
                FLASH_read(MpuOffset, 6);
                s_mpu_inited = 1U;
                MPU6050_Unlock();
                return MPU6050_OK;
            }
        }
        MPU6050_DelayMs(pdMS_TO_TICKS(10));
    }
    MPU6050_Unlock();
    return MPU6050_ERR_I2C;
}

/* ============================================================
 * 读取并更新缓存
 * ============================================================ */
MPU6050_Status_t MpuGetData(void)
{
    MPU6050_Status_t ret;
    int16_t raw[6];
    uint8_t i;

    if(s_mpu_inited == 0U)
    {
        return MPU6050_ERR_NOT_INIT;
    }

    ret = MPU6050_Lock(MPU_I2C_TIMEOUT_MS);
    if(ret != MPU6050_OK)
    {
        return ret;
    }

    ret = MPU6050_ReadRawBlock(raw);
    if(ret != MPU6050_OK)
    {
        MPU6050_Unlock();
        return ret;
    }

    // 处理数据
    for(i = 0; i < 6; ++i)
    {
        int32_t v = (int32_t)raw[i] - (int32_t)MpuOffset[i];

        if(i < 3)
        {
            /* 加速度，一维卡尔曼 */
            float out = Kalman1D_Update(&s_ekf[i], (float)v);
            v = (int32_t)out;
        }
        else
        {
            /* 陀螺仪：一阶低通 */
            uint8_t k = (uint8_t)(i - 3);
            s_gyro_lpf[k] = s_gyro_lpf[k] * (1.0f - MPU_GYRO_LPF_FACTOR) + (float)v * MPU_GYRO_LPF_FACTOR;
            v = (int32_t)s_gyro_lpf[k];
        }
        // 进行赋值
        switch (i)
        {
            case 0: s_mpu_raw.accX  = (int16_t)v; break;
            case 1: s_mpu_raw.accY  = (int16_t)v; break;
            case 2: s_mpu_raw.accZ  = (int16_t)v; break;
            case 3: s_mpu_raw.gyroX = (int16_t)v; break;
            case 4: s_mpu_raw.gyroY = (int16_t)v; break;
            case 5: s_mpu_raw.gyroZ = (int16_t)v; break;
            default: break;
        }
    }

    MPU6050_Unlock();
    return MPU6050_OK;
}

/* ============================================================
 * 校准
 * 说明：
 * 1. 这个函数只建议在初始化阶段调用一次
 * 2. 调用前，上层应该暂停姿态控制 / 电机输出 / 周期任务
 * 3. 不要在这里强行关 TIM3 中断，交给上层控制
 * ============================================================ */
MPU6050_Status_t MpuGetOffset(void)
{
    MPU6050_Status_t ret;
    int16_t raw[6];
    int16_t last_gyro[3] ={0};
    int16_t err_gyro[3] = {0};
    int64_t sum[6] = {0};
    uint16_t stable_count = 0;
    uint16_t discard_count = 0;
    uint16_t collect_count = 0;
    TickType_t start_tick;

    if(s_mpu_inited == 0U)
    {
        return MPU6050_ERR_NOT_INIT;
    }

    ret = MPU6050_Lock(pdMS_TO_TICKS(MPU_I2C_TIMEOUT_MS));
    if(ret != MPU6050_OK)
    {
        return ret;
    }

    memset(MpuOffset, 0, sizeof(MpuOffset));
    MpuOffset[2] = 8192;  // 4g量程下，静止时z轴约为1g

    start_tick = xTaskGetTickCount();

    /* 先等待陀螺仪进入静止状态 */
    while(stable_count < MPU_CALIB_STABLE_STEP)
    {
        ret = MPU6050_ReadRawBlock(raw);
        if(ret != MPU6050_OK)
        {
            MPU6050_Unlock();
            return ret;
        }

        err_gyro[0] = raw[3] - last_gyro[0];
        err_gyro[1] = raw[4] - last_gyro[1];
        err_gyro[2] = raw[5] - last_gyro[2];

        last_gyro[0] = raw[3];
        last_gyro[1] = raw[4];
        last_gyro[2] = raw[5];

        if ((err_gyro[0] >= MPU_CALIB_STABLE_MIN && err_gyro[0] <= MPU_CALIB_STABLE_STEP) &&
            (err_gyro[1] >= MPU_CALIB_STABLE_MIN && err_gyro[1] <= MPU_CALIB_STABLE_STEP) &&
            (err_gyro[2] >= MPU_CALIB_STABLE_MIN && err_gyro[2] <= MPU_CALIB_STABLE_STEP))
        {
            stable_count++;
        }
        else
        {
            stable_count = 0;
        }

        if ((xTaskGetTickCount() - start_tick) > pdMS_TO_TICKS(MPU_CALIB_MAX_WAIT_MS))
        {
            MPU6050_Unlock();
            return MPU6050_ERR_TIMEOUT;
        }

        MPU6050_DelayMs(MPU_CALIB_SAMPLE_DELAY_MS);
    }

    /* 丢弃前 100 组，避免刚稳定时数据抖动 */
    while(discard_count < MPU_CALIB_DISCARD_COUNT)
    {
        ret = MPU6050_ReadRawBlock(raw);
        if(ret != MPU6050_OK)
        {
            MPU6050_Unlock();
            return ret;
        }
        discard_count++;
        MPU6050_DelayMs(pdMS_TO_TICKS(MPU_CALIB_SAMPLE_DELAY_MS));
    }

    /* 再采集 256 组，做平均 */
    while(collect_count < MPU_CALIB_COLLECT_COUNT)
    {
        ret = MPU6050_ReadRawBlock(raw);
        if (ret != MPU6050_OK)
        {
            MPU6050_Unlock();
            return ret;
        }

        sum[0] += raw[0];
        sum[1] += raw[1];
        sum[2] += raw[2];
        sum[3] += raw[3];
        sum[4] += raw[4];
        sum[5] += raw[5];

        collect_count++;
        MPU6050_DelayMs(pdMS_TO_TICKS(MPU_CALIB_SAMPLE_DELAY_MS));
    }

    /* 计算偏移 */
    MpuOffset[0] = (int16_t)(sum[0] / MPU_CALIB_COLLECT_COUNT);
    MpuOffset[1] = (int16_t)(sum[1] / MPU_CALIB_COLLECT_COUNT);
    MpuOffset[2] = (int16_t)((sum[2] / MPU_CALIB_COLLECT_COUNT) - 8192);  // 有疑问？

    MpuOffset[3] = (int16_t)(sum[3] / MPU_CALIB_COLLECT_COUNT);
    MpuOffset[4] = (int16_t)(sum[4] / MPU_CALIB_COLLECT_COUNT);
    MpuOffset[5] = (int16_t)(sum[5] / MPU_CALIB_COLLECT_COUNT);

    FLASH_write(MpuOffset, 6);

    MPU6050_Unlock();
    return MPU6050_OK;

}


/* ============================================================
 * 线程安全读取接口
 * ============================================================ */
MPU6050_Status_t get_MPU6050_datas(_st_Mpu *mpu)
{
    MPU6050_Status_t ret;
    if(mpu == NULL)
    {
        return MPU6050_ERR_PARAM;
    }

    ret = MPU6050_Lock(MPU_I2C_TIMEOUT_MS);
    if(ret != MPU6050_OK)
    {
        return ret;
    }

    *mpu = s_mpu_raw;

    MPU6050_Unlock();
    return MPU6050_OK;
}

MPU6050_Status_t get_MPU6050_Angle(_st_AngE *pAngE)
{
    MPU6050_Status_t ret;

    if (pAngE == NULL)
    {
        return MPU6050_ERR_PARAM;
    }

    ret = MPU6050_Lock(pdMS_TO_TICKS(MPU_I2C_TIMEOUT_MS));
    if (ret != MPU6050_OK)
    {
        return ret;
    }

    *pAngE = s_angle;

    MPU6050_Unlock();
    return MPU6050_OK;
}


MPU6050_Status_t set_MPU6050_Angle(const _st_AngE *pAngE)
{
    MPU6050_Status_t ret;

    if (pAngE == NULL)
    {
        return MPU6050_ERR_PARAM;
    }

    ret = MPU6050_Lock(pdMS_TO_TICKS(MPU_I2C_TIMEOUT_MS));
    if (ret != MPU6050_OK)
    {
        return ret;
    }

    s_angle = *pAngE;

    MPU6050_Unlock();
    return MPU6050_OK;
}