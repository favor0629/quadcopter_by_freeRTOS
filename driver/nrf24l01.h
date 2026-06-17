#ifndef __NRF24L01_H
#define __NRF24L01_H

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <stdint.h>

/* ============================================================
 * nRF24L01 基本参数
 * ============================================================ */
#define TX_ADR_WIDTH        5U
#define RX_ADR_WIDTH        5U
#define TX_PLOAD_WIDTH      32U
#define RX_PLOAD_WIDTH      32U

/* ============================================================
 * nRF24L01 SPI 命令
 * ============================================================ */
#define SPI_READ_REG        0x00U
#define SPI_WRITE_REG       0x20U
#define RD_RX_PLOAD         0x61U
#define WR_TX_PLOAD         0xA0U
#define FLUSH_TX            0xE1U
#define FLUSH_RX            0xE2U
#define REUSE_TX_PL         0xE3U
#define NOP                 0xFFU

/* ============================================================
 * nRF24L01 寄存器地址
 * ============================================================ */
#define CONFIG              0x00U
#define NCONFIG             CONFIG
#define EN_AA               0x01U
#define EN_RXADDR           0x02U
#define SETUP_AW            0x03U
#define SETUP_RETR          0x04U
#define RF_CH               0x05U
#define RF_SETUP            0x06U
#define STATUS              0x07U
#define OBSERVE_TX          0x08U
#define CD                  0x09U
#define RX_ADDR_P0          0x0AU
#define RX_ADDR_P1          0x0BU
#define RX_ADDR_P2          0x0CU
#define RX_ADDR_P3          0x0DU
#define RX_ADDR_P4          0x0EU
#define RX_ADDR_P5          0x0FU
#define TX_ADDR             0x10U
#define RX_PW_P0            0x11U
#define RX_PW_P1            0x12U
#define RX_PW_P2            0x13U
#define RX_PW_P3            0x14U
#define RX_PW_P4            0x15U
#define RX_PW_P5            0x16U
#define FIFO_STATUS         0x17U

/* ============================================================
 * STATUS 位定义
 * ============================================================ */
#define MAX_TX              0x10U
#define TX_OK               0x20U
#define RX_OK               0x40U

/* ============================================================
 * RTOS 版返回值
 * ============================================================ */
typedef enum
{
    NRF24L01_OK = 0U,
    NRF24L01_ERROR = 1U,
    NRF24L01_TIMEOUT = 2U,
    NRF24L01_BUSY = 3U,
    NRF24L01_NOT_INIT = 4U,
    NRF24L01_INVALID_PARAM = 5U,
    NRF24L01_MAX_RT = 6U
} NRF24L01_Status_t;

/* ============================================================
 * 对外接口
 * ============================================================ */
void NRF24L01_Configuration(void);
void NRF24L01_EXTI_Init(void);

NRF24L01_Status_t NRF24L01_Init(void);

/* 模式切换 */
void NRF24L01_RX_Mode(void);
void NRF24L01_TX_Mode(void);

/* 底层寄存器操作 */
uint8_t NRF24L01_Write_Buf(uint8_t regaddr, const uint8_t *pBuf, uint8_t datalen);
uint8_t NRF24L01_Read_Buf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen);
uint8_t NRF24L01_Read_Reg(uint8_t regaddr, uint8_t *rx);
uint8_t NRF24L01_Write_Reg(uint8_t regaddr, uint8_t data);

/* 检查模块在位 */
uint8_t NRF24L01_Check(void);

/* 发送/接收 */
NRF24L01_Status_t NRF24L01_TxPacket(uint8_t *txbuf);

/* 非阻塞接收：轮询当前是否有包 */
NRF24L01_Status_t NRF24L01_RxPacket(uint8_t *rxbuf);

/* 阻塞接收：等待 IRQ 唤醒，带超时 */
NRF24L01_Status_t NRF24L01_WaitRxPacket(uint8_t *rxbuf, TickType_t timeout);

/* 在 EXTI 中断里调用 */
void NRF24L01_IRQHandlerFromISR(BaseType_t *pxHigherPriorityTaskWoken);

#endif