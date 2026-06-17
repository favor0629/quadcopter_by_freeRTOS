#include "iic.h"
#include "Delay.h"

#include "FreeRTOS.h"
#include "semphr.h"


/* ========== 参数配置 ========== */
#define IIC_MUTEX_TIMEOUT_MS     20U
#define IIC_SCL_TIMEOUT_US       100U

#define IIC_DELAY_LOW_US          3U
#define IIC_DELAY_HIGH_US         5U


/* ========== 互斥锁 ========== */
static StaticSemaphore_t s_iic_mutex_buf;
static SemaphoreHandle_t s_iic_mutex = NULL;
static uint8_t s_iic_inited = 0U;


/* ========== 内部延时 ========== */
static void IIC_DelayLow(void)
{
    Delay_us(IIC_DELAY_LOW_US);
}

static void IIC_DelayHigh(void)
{
    Delay_us(IIC_DELAY_HIGH_US);
}

/* ========== 互斥保护 ========== */
static IIC_Status_t IIC_Lock(TickType_t timeout)
{
    if(s_iic_inited == 0U || s_iic_mutex == NULL)
    {
        return IIC_ERR_NOT_INIT;
    }

    if(xSemaphoreTake(s_iic_mutex, timeout) != pdTRUE)
    {
        return IIC_ERR_TIMEOUT;  // 获取锁超时
    }

    // 获取锁成功
    return IIC_OK;
}




static IIC_Status_t IIC_Unlock(void)
{
    if(s_iic_inited == 0U || s_iic_mutex == NULL)
    {
        return IIC_ERR_NOT_INIT;
    }
    if(xSemaphoreGive(s_iic_mutex) != pdTRUE)
    {
        return IIC_ERR;
    }
    return IIC_OK;
}

/* ========== 线状态等待 ========== */
static IIC_Status_t IIC_WaitSCLHigh(uint32_t timeout_us)
{
    while(!SCL_read())
    {
        if(timeout_us == 0U)
        {
            return IIC_ERR_TIMEOUT;
        }

        timeout_us--;
        Delay_us(1);
    }
    return IIC_OK;
}

static IIC_Status_t IIC_Start(void)
{
    SDA_H();
    SCL_H();
    IIC_DelayHigh();

    if(IIC_WaitSCLHigh(IIC_SCL_TIMEOUT_US) != IIC_OK)
    {
        return IIC_ERR_TIMEOUT;
    }

    if(!SDA_read())
    {
        return IIC_ERR_BUSY;
    }

    SDA_L();  // 产生下降沿
    IIC_DelayLow();

    if(SDA_read())
    {
        return IIC_ERR;
    }

    SCL_L();
    IIC_DelayLow();
    return IIC_OK;
}

static IIC_Status_t IIC_Stop(void)
{
    SCL_L();
    IIC_DelayLow();

    SDA_L();
    IIC_DelayLow();

    SCL_H();
    if(IIC_WaitSCLHigh(IIC_SCL_TIMEOUT_US) != IIC_OK)
    {
        return IIC_ERR_TIMEOUT;
    }

    IIC_DelayHigh();
    SDA_H();
    IIC_DelayHigh();

    return IIC_OK;
}

/* ========== ACK/NACK ========== */
static IIC_Status_t IIC_Ack(void)
{
    SCL_L();
    IIC_DelayLow();

    SDA_L();
    IIC_DelayLow();

    SCL_H();
    if(IIC_WaitSCLHigh(IIC_SCL_TIMEOUT_US) != IIC_OK)
    {
        return IIC_ERR_TIMEOUT;
    }

    IIC_DelayHigh();

    SCL_L();
    SDA_H();
    IIC_DelayLow();

    return IIC_OK;

}

static IIC_Status_t IIC_NoAck(void)
{
    SCL_L();
    IIC_DelayLow();

    SDA_H();
    IIC_DelayLow();

    SCL_H();
    if(IIC_WaitSCLHigh(IIC_SCL_TIMEOUT_US) != IIC_OK)
    {
        return IIC_ERR_TIMEOUT;
    }

    IIC_DelayHigh();

    SCL_L();
    SDA_H();
    IIC_DelayLow();

    return IIC_OK;
}

static IIC_Status_t IIC_WaitAck(void)
{
    SCL_L();
    IIC_DelayLow();

    SDA_H();
    IIC_DelayLow();

    SCL_H();
    if (IIC_WaitSCLHigh(IIC_SCL_TIMEOUT_US) != IIC_OK)
    {
        return IIC_ERR_TIMEOUT;
    }

    IIC_DelayHigh();

    if (SDA_read())
    {
        SCL_L();
        SDA_H();
        IIC_DelayLow();
        return IIC_ERR_NACK;
    }

    SCL_L();
    SDA_H();
    IIC_DelayLow();

    return IIC_OK;
}


/* ========== 发送/接收字节 ========== */
static IIC_Status_t IIC_SendByte(uint8_t byte)
{
    uint8_t i;
    for(i = 0; i < 8; ++i)
    {
        SCL_L();
        IIC_DelayLow();

        if(byte & 0x80U)
        {
            SDA_H();
        }
        else
        {
            SDA_L();
        }
        byte <<= 1;
        IIC_DelayLow();

        SCL_H();
        if(IIC_WaitSCLHigh(IIC_SCL_TIMEOUT_US) != IIC_OK)
        {
            return IIC_ERR_TIMEOUT;
        }
        IIC_DelayHigh();
    }

    SCL_L();
    SDA_H();
    IIC_DelayLow();
    return IIC_OK;
}

static IIC_Status_t IIC_ReadByte(uint8_t *byte)
{
    uint8_t i;

    if(byte == NULL)
    {
        return IIC_ERR_PARAM;
    }

    *byte = 0;  // 初始为0

    /* 释放数据线 */
    SDA_H();

    for(i = 0; i < 8; ++i)
    {
        *byte <<= 1;
        SCL_L();
        IIC_DelayLow();

        SCL_H();
        if(IIC_WaitSCLHigh(IIC_SCL_TIMEOUT_US) != IIC_OK)
        {
            return IIC_ERR_TIMEOUT;
        }
        IIC_DelayHigh();

        if(SDA_read())
        {
            *byte |= 0x01U;
        }
    }

    SCL_L();
    SDA_H();
    IIC_DelayLow();

    return IIC_OK;
}


/* ========== 总线恢复 ========== */
static void IIC_BusRecovery(void)
{
    uint8_t i;

    SDA_H();
    SCL_H();
    Delay_us(5);

    if(!SDA_read())
    {
        for(i = 0; i < 9; ++i)
        {
            SCL_L();
            Delay_us(5);
            SCL_H();
            if(IIC_WaitSCLHigh(IIC_SCL_TIMEOUT_US) != IIC_OK)
            {
                break;
            }

            Delay_us(5);
        }
    }

    IIC_Stop();
}


/* ========== 初始化 ========== */
void IIC_Init(void)
{
    GPIO_InitTypeDef gpio_init_structure;
    RCC_APB2PeriphClockCmd(IIC_RCC, ENABLE);

    gpio_init_structure.GPIO_Pin = SCL_PIN | SDA_PIN;
    gpio_init_structure.GPIO_Mode = GPIO_Mode_Out_OD;
    gpio_init_structure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(IIC_GPIO, &gpio_init_structure);

    SCL_H();
    SDA_H();
    Delay_us(5);

#if (configSUPPORT_STATIC_ALLOCATION == 1)
    s_iic_mutex = xSemaphoreCreateMutexStatic(&s_iic_mutex_buf);
#else
     s_iic_mutex = xSemaphoreCreateMutex();
#endif


    s_iic_inited = 1U;

    IIC_BusRecovery();
}

/* ========== 写单字节 ========== */
IIC_Status_t IIC_Write_One_Byte(uint8_t addr, uint8_t reg, uint8_t data)
{
    return IIC_Write_Bytes(addr, reg, &data, 1);
}

/* ========== 读单字节 ========== */
IIC_Status_t IIC_Read_One_Byte(uint8_t addr, uint8_t reg, uint8_t *data)
{
    return IIC_Read_Bytes(addr, reg, data, 1);
}

/* ========== 写多字节 ========== */
IIC_Status_t IIC_Write_Bytes(uint8_t addr, uint8_t reg, const uint8_t *data, uint16_t len)
{
    IIC_Status_t ret;

    if (data == NULL || len == 0U)
        return IIC_ERR_PARAM;

    ret = IIC_Lock(pdMS_TO_TICKS(IIC_MUTEX_TIMEOUT_MS));
    if (ret != IIC_OK)
        return ret;

    ret = IIC_Start();
    if (ret != IIC_OK)
        goto out;

    ret = IIC_SendByte((uint8_t)((addr << 1) | 0x00U));
    if (ret != IIC_OK)
        goto stop_out;

    ret = IIC_WaitAck();
    if (ret != IIC_OK)
        goto stop_out;

    ret = IIC_SendByte(reg);
    if (ret != IIC_OK)
        goto stop_out;

    ret = IIC_WaitAck();
    if (ret != IIC_OK)
        goto stop_out;

    while (len--)
    {
        ret = IIC_SendByte(*data++);
        if (ret != IIC_OK)
            goto stop_out;

        ret = IIC_WaitAck();
        if (ret != IIC_OK)
            goto stop_out;
    }

stop_out:
    IIC_Stop();

out:
    IIC_Unlock();
    return ret;
}


/* ========== 读多字节 ========== */
IIC_Status_t IIC_Read_Bytes(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len)
{
    IIC_Status_t ret;

    if (data == NULL || len == 0U)
        return IIC_ERR_PARAM;

    ret = IIC_Lock(pdMS_TO_TICKS(IIC_MUTEX_TIMEOUT_MS));
    if (ret != IIC_OK)
        return ret;

    ret = IIC_Start();
    if (ret != IIC_OK)
        goto out;

    ret = IIC_SendByte((uint8_t)((addr << 1) | 0x00U));
    if (ret != IIC_OK)
        goto stop_out;

    ret = IIC_WaitAck();
    if (ret != IIC_OK)
        goto stop_out;

    ret = IIC_SendByte(reg);
    if (ret != IIC_OK)
        goto stop_out;

    ret = IIC_WaitAck();
    if (ret != IIC_OK)
        goto stop_out;

    ret = IIC_Start();   /* repeated start */
    if (ret != IIC_OK)
        goto stop_out;

    ret = IIC_SendByte((uint8_t)((addr << 1) | 0x01U));
    if (ret != IIC_OK)
        goto stop_out;

    ret = IIC_WaitAck();
    if (ret != IIC_OK)
        goto stop_out;

    while (len--)
    {
        ret = IIC_ReadByte(data++);
        if (ret != IIC_OK)
        {
            goto stop_out;
        }
        if (len == 0U)
        {
            ret = IIC_NoAck();
        }
        else
        {
            ret = IIC_Ack();
        }
        
        if (ret != IIC_OK)
            goto stop_out;
    }

stop_out:
    IIC_Stop();

out:
    IIC_Unlock();
    return ret;
}