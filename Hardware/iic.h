#ifndef _I2C_H_
#define _I2C_H_

#include "stm32f10x.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 硬件配置 ========== */
#define IIC_RCC       RCC_APB2Periph_GPIOB
#define IIC_GPIO      GPIOB

#define SCL_PIN       GPIO_Pin_6
#define SDA_PIN       GPIO_Pin_7

/* ========== 线控宏 ========== */
#define SCL_H()       (IIC_GPIO->BSRR = SCL_PIN)
#define SCL_L()       (IIC_GPIO->BRR  = SCL_PIN)

#define SDA_H()       (IIC_GPIO->BSRR = SDA_PIN)
#define SDA_L()       (IIC_GPIO->BRR  = SDA_PIN)

#define SCL_read()    (IIC_GPIO->IDR & SCL_PIN)
#define SDA_read()    (IIC_GPIO->IDR & SDA_PIN)

/* ========== 状态码 ========== */
typedef enum
{
    IIC_OK = 0,
    IIC_ERR,
    IIC_ERR_TIMEOUT,
    IIC_ERR_BUSY,
    IIC_ERR_NACK,
    IIC_ERR_NOT_INIT,
    IIC_ERR_PARAM
} IIC_Status_t;

/* ========== 对外接口 ========== */
void IIC_Init(void);

IIC_Status_t IIC_Write_One_Byte(uint8_t addr, uint8_t reg, uint8_t data);
IIC_Status_t IIC_Read_One_Byte(uint8_t addr, uint8_t reg, uint8_t *data);

IIC_Status_t IIC_Write_Bytes(uint8_t addr, uint8_t reg, const uint8_t *data, uint16_t len);
IIC_Status_t IIC_Read_Bytes(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len);

/* 兼容你原来的命名 */
#define IIC_read_Bytes IIC_Read_Bytes

#ifdef __cplusplus
}
#endif

#endif