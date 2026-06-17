#ifndef __SPI_H
#define __SPI_H

#include "stm32f10x.h"
#include <stdint.h>

#include "FreeRTOS.h"
#include "semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================
 * GPIO 配置
 * ========================= */
#define SPI_GPIO_RCC      RCC_APB2Periph_GPIOB
#define SPI_GPIO_PORT     GPIOB

#define SPI_SCK_PIN       GPIO_Pin_13
#define SPI_MISO_PIN      GPIO_Pin_14
#define SPI_MOSI_PIN      GPIO_Pin_15
#define SPI_CS_PIN        GPIO_Pin_12

/* =========================
 * 线控宏
 * ========================= */
#define SPI_SCK_H()       (SPI_GPIO_PORT->BSRR = SPI_SCK_PIN)
#define SPI_SCK_L()       (SPI_GPIO_PORT->BRR  = SPI_SCK_PIN)

#define SPI_MOSI_H()      (SPI_GPIO_PORT->BSRR = SPI_MOSI_PIN)
#define SPI_MOSI_L()      (SPI_GPIO_PORT->BRR  = SPI_MOSI_PIN)

#define SPI_CS_H()        (SPI_GPIO_PORT->BSRR = SPI_CS_PIN)
#define SPI_CS_L()        (SPI_GPIO_PORT->BRR  = SPI_CS_PIN)

#define SPI_MISO_READ()   ((SPI_GPIO_PORT->IDR & SPI_MISO_PIN) ? 1U : 0U)

/* =========================
 * 状态码
 * ========================= */
typedef enum
{
    SPI_OK = 0,
    SPI_ERR,
    SPI_ERR_TIMEOUT,
    SPI_ERR_BUSY,
    SPI_ERR_NOT_INIT,
    SPI_ERR_PARAM
} SPI_Status_t;

/* =========================
 * 对外接口
 * ========================= */
void SPI_Config(void);

/* 事务级互斥 */
SPI_Status_t SPI_Lock(TickType_t timeout_ms);
SPI_Status_t SPI_Unlock(void);

/* CS 控制 */
void SPI_CS_Select(void);
void SPI_CS_Deselect(void);

/* 单字节收发，要求已加锁，且 CS 已经拉低 */
uint8_t SPI_RW_Byte(uint8_t byte);

/* 单字节安全封装：内部自动加锁/解锁 */
SPI_Status_t SPI_ExchangeByte(uint8_t tx, uint8_t *rx);

/* 多字节安全封装：内部自动加锁/解锁，自动控制 CS */
SPI_Status_t SPI_Transfer(const uint8_t *tx, uint8_t *rx, uint16_t len);

/* 只发不收 */
SPI_Status_t SPI_WriteBytes(const uint8_t *tx, uint16_t len);

/* 只收不发，发送填充值通常用 0xFF */
SPI_Status_t SPI_ReadBytes(uint8_t fill, uint8_t *rx, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif