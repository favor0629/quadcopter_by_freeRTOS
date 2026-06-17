#ifndef __USART_H
#define __USART_H

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USART1_RX_QUEUE_LEN          128U
#define USART1_MUTEX_TIMEOUT_MS      20U

/* 这个优先级要按你的 FreeRTOS 配置调整
 * 只要 USART1_IRQHandler 里用了 xQueueSendFromISR，
 * 这个中断优先级就不能高到超过 FreeRTOS 允许调用 FromISR API 的上限。
 */
#define USART1_IRQ_PREEMPTION_PRIORITY   6U
#define USART1_IRQ_SUB_PRIORITY          0U

typedef enum
{
    USART_OK = 0,
    USART_ERR,
    USART_ERR_TIMEOUT,
    USART_ERR_BUSY,
    USART_ERR_NOT_INIT,
    USART_ERR_PARAM
} USART_Status_t;

void USART1_SendRawByte(uint8_t ch);


//void USART1_SendByte(const int8_t *Data,uint8_t len);

void USART1_Config(uint32_t baudRate);
void USART1_SetBaudRate(uint32_t baudRate);

/* 兼容你原来的接口 */
void USART1_SendByte(const int8_t *Data, uint8_t len);

/* 推荐新接口 */
USART_Status_t USART1_SendBuffer(const uint8_t *data, uint16_t len);

/* 单字节发送 */
USART_Status_t USART1_SendChar(uint8_t ch);

/* 接收接口 */
BaseType_t USART1_ReadByte(uint8_t *ch, TickType_t timeout_ticks);
uint16_t USART1_ReadBuffer(uint8_t *buf, uint16_t len, TickType_t timeout_ticks);

/* RX 缓冲状态 */
uint16_t USART1_RxPending(void);
void USART1_ClearRxBuffer(void);
uint32_t USART1_GetRxOverflowCount(void);

/* 中断函数 */
void USART1_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif

