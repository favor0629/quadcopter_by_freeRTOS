#include "usart.h"
#include "misc.h"
#include "stdio.h"
#include "task.h"
#include <string.h>

/* ============================================================
 * USART1 TX DMA 环形缓冲区
 * ============================================================ */
/* 发送缓冲区大小，必须是2的幂 */
#ifndef USART1_TX_BUF_SIZE
#define USART1_TX_BUF_SIZE      1024U
#endif

#if ((USART1_TX_BUF_SIZE & (USART1_TX_BUF_SIZE - 1U)) != 0U)
#error USART1_TX_BUF_SIZE must be a power of 2
#endif

#define USART1_TX_BUF_MASK     (USART1_TX_BUF_SIZE - 1U)

/* 发送缓冲 */
static uint8_t s_tx_buf[USART1_TX_BUF_SIZE];

/* 头尾索引 */
static volatile uint16_t s_tx_head = 0U;
static volatile uint16_t s_tx_tail = 0U;

/* DMA 当前是否忙 */
static volatile uint8_t s_tx_dma_busy = 0U;

/* 本次DMA发送长度， 用于TC中断后推进tail */
static volatile uint16_t s_tx_dma_len = 0U;

/* 丢弃字节计数 */
static volatile uint32_t s_tx_drop_cnt = 0U;

/* =========================
 * 内部状态
 * ========================= */
static StaticSemaphore_t s_usart_mutex_buf;
static SemaphoreHandle_t s_usart_mutex = NULL;


static StaticQueue_t s_rx_queue_buf;
static uint8_t s_rx_queue_storage[USART1_RX_QUEUE_LEN];
static QueueHandle_t s_rx_queue = NULL;


static volatile uint8_t s_usart_inited = 0U;
static volatile uint32_t s_rx_overflow_cnt = 0U;
static uint32_t s_baudrate = 115200;




/* =========================
 * 内部工具
 * ========================= */
static inline uint16_t USART1_TxNext(uint16_t index)
{
    return (uint16_t)((index + 1) & USART1_TX_BUF_MASK);
}

/*负责把环形缓冲区中“当前连续的一段”交给 DMA 发送*/
static void USART1_TxDmaKick(void)
{
    uint16_t len;
    if(s_tx_dma_busy != 0U)
    {
        return;
    }
    if(s_tx_head == s_tx_tail)
    {
        return;
    }

    /* 只启动一段连续区域 */
    if(s_tx_head > s_tx_tail)
    {
        len = (uint16_t)(s_tx_head - s_tx_tail);
    }
    else
    {
        len = (uint16_t)(USART1_TX_BUF_SIZE - s_tx_tail);
    }

    s_tx_dma_busy = 1U;
    s_tx_dma_len = len;

    DMA_Cmd(DMA1_Channel4, DISABLE);
    DMA_ClearITPendingBit(DMA1_IT_GL4);

    /* 重新装载本次DMA传输参数 */
    DMA1_Channel4->CMAR = (uint32_t)&s_tx_buf[s_tx_tail];
    DMA1_Channel4->CNDTR = len;

    /* 开始发送 */
    DMA_Cmd(DMA1_Channel4, ENABLE);
}

 static USART_Status_t USART1_Lock(TickType_t timeout_ms)
{
    if(s_usart_inited == 0U || s_usart_mutex == NULL)
    {
        return USART_ERR_NOT_INIT;
    }
    if(xSemaphoreTake(s_usart_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
    {
        return USART_ERR_TIMEOUT;
    }

    return USART_OK;
}


static USART_Status_t USART1_Unlock(void)
{
    // if(s_usart_mutex != NULL)
    // {
    //     xSemaphoreGive(s_usart_mutex);
    // }
    // return USART_OK;
    if(s_usart_mutex == NULL)
    {
        return USART_ERR_NOT_INIT;
    }

    if(xSemaphoreGive(s_usart_mutex) != pdTRUE)
    {
        return USART_ERR;
    }
    return USART_OK;
}

static void USART1_ApplyConfig(uint32_t baudrate)
{
    USART_InitTypeDef usart_init_structure;

    USART_Cmd(USART1, DISABLE);

    USART_StructInit(&usart_init_structure);

    usart_init_structure.USART_BaudRate = baudrate;
    usart_init_structure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init_structure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart_init_structure.USART_Parity = USART_Parity_No;
    usart_init_structure.USART_StopBits = USART_StopBits_1;
    usart_init_structure.USART_WordLength = USART_WordLength_8b;

    USART_Init(USART1, &usart_init_structure);

    /* 开启rx中断 */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    USART_Cmd(USART1, ENABLE);
}

void USART1_SendRawByte(uint8_t ch)
{
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
        /* busy wait */
    }
    USART_SendData(USART1, (uint16_t)ch);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    {
        /* wait until frame fully sent */
    }
}



// void USART1_SendByte(const int8_t *Data,uint8_t len)
// { 
// 	uint8_t i;
	
// 	for(i=0;i<len;i++)
// 	{
// 		while (!(USART1->SR & USART_FLAG_TXE));	 // while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
// 		USART_SendData(USART1,*(Data+i));	 
// 	}
// }


/* =========================
 * 初始化
 * ========================= */
void USART1_Config(uint32_t baudRate)
{
    // 如果已经初始化，退出
    if(s_usart_inited != 0U)
    {
        return;
    }

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);


    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitTypeDef gpio_init_strcutrue;

    // tx:PA9
    gpio_init_strcutrue.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_init_strcutrue.GPIO_Pin = GPIO_Pin_9;
    gpio_init_strcutrue.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio_init_strcutrue);

    // rx:PA10
    gpio_init_strcutrue.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    gpio_init_strcutrue.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &gpio_init_strcutrue);


    NVIC_InitTypeDef nvic_init_strcuture;
    nvic_init_strcuture.NVIC_IRQChannel = USART1_IRQn;
    nvic_init_strcuture.NVIC_IRQChannelCmd = ENABLE;
    nvic_init_strcuture.NVIC_IRQChannelPreemptionPriority = USART1_IRQ_PREEMPTION_PRIORITY;
    nvic_init_strcuture.NVIC_IRQChannelSubPriority = USART1_IRQ_SUB_PRIORITY;

    NVIC_Init(&nvic_init_strcuture);

    /* DMA1_Channel4中断：USART_Tx */
    nvic_init_strcuture.NVIC_IRQChannel = DMA1_Channel4_IRQn;
    nvic_init_strcuture.NVIC_IRQChannelCmd = ENABLE;
    nvic_init_strcuture.NVIC_IRQChannelPreemptionPriority = USART1_IRQ_PREEMPTION_PRIORITY;
    nvic_init_strcuture.NVIC_IRQChannelSubPriority = USART1_IRQ_SUB_PRIORITY;

    NVIC_Init(&nvic_init_strcuture);



    /* 先启用基本的 UART 硬件，即使互斥量创建失败也能进行裸发送 */
    s_baudrate = baudRate;
    //USART1_ApplyConfig(s_baudrate);

    USART_InitTypeDef usart_init_structure;
    USART_StructInit(&usart_init_structure);
    usart_init_structure.USART_BaudRate = s_baudrate;
    usart_init_structure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init_structure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    usart_init_structure.USART_Parity = USART_Parity_No;
    usart_init_structure.USART_StopBits = USART_StopBits_1;
    usart_init_structure.USART_WordLength = USART_WordLength_8b;

    USART_Init(USART1, &usart_init_structure);

    /* 开启rxne中断 */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /* Tx DMA 请求打开 */
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

    /* DMA1_Channel4 配置 */
    DMA_InitTypeDef dma_init_structure;
    //DMA_StructInit(&dma_init_structure);
    DMA_DeInit(DMA1_Channel4);
    dma_init_structure.DMA_PeripheralBaseAddr =(uint32_t)&USART1->DR;
    dma_init_structure.DMA_MemoryBaseAddr = (uint32_t)s_tx_buf;
    dma_init_structure.DMA_DIR = DMA_DIR_PeripheralDST; //方向： DR -> s_tx_buf
    dma_init_structure.DMA_BufferSize = 1U;
    dma_init_structure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma_init_structure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma_init_structure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma_init_structure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma_init_structure.DMA_Mode = DMA_Mode_Normal;
    dma_init_structure.DMA_Priority = DMA_Priority_High;
    dma_init_structure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4, &dma_init_structure);

    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
    DMA_Cmd(DMA1_Channel4, DISABLE);

    USART_Cmd(USART1, ENABLE);

    #if(configSUPPORT_STATIC_ALLOCATION == 1)
        s_usart_mutex = xSemaphoreCreateMutexStatic(&s_usart_mutex_buf);
        s_rx_queue = xQueueCreateStatic(USART1_RX_QUEUE_LEN,
        sizeof(uint8_t), s_rx_queue_storage, &s_rx_queue_buf);
    
    #else
        s_usart_mutex = xSemaphoreCreateMutex();
        s_rx_queue = xQueueCreate(USART1_RX_QUEUE_LEN, sizeof(uint8_t));
    #endif

    /* 即使互斥量或队列创建失败，UART 硬件已启用，裸发送能工作 */
    if(s_usart_mutex == NULL || s_rx_queue == NULL)
    {
        /* 注意：如果到这里，互斥保护的发送会失败，但裸发送仍然可用 */
    }



    s_usart_inited = 1U;
}

/* =========================
 * 改波特率
 * ========================= */
void USART1_SetBaudRate(uint32_t baudRate)
{
    USART_Status_t ret;

    ret = USART1_Lock(USART1_MUTEX_TIMEOUT_MS);
    if(ret != USART_OK)
    {
        return;
    }
    s_baudrate = baudRate;
    USART1_ApplyConfig(s_baudrate);

    USART1_Unlock();
}


/* =========================
 * 发送单字节
 * ========================= */
USART_Status_t USART1_SendChar(uint8_t ch)
{
    USART_Status_t ret;

    ret = USART1_Lock(USART1_MUTEX_TIMEOUT_MS);
    if(ret != USART_OK)
    {
        return ret;
    }

    USART1_SendRawByte(ch);

    USART1_Unlock();

    return USART_OK;
}


/* =========================
 * 发送缓冲区
 * ========================= */
USART_Status_t USART1_SendBuffer(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    uint16_t next;

    if(data == NULL || len == 0U)
    {
        return USART_ERR_PARAM;
    }
    if(s_usart_inited == 0U)
    {
        return USART_ERR_NOT_INIT;
    }

    taskENTER_CRITICAL();
    {
        for(i = 0U; i < len; ++i)
        {
            next = USART1_TxNext(s_tx_head);

            /* 缓冲区满， 丢弃后续数据 */
            if(next == s_tx_tail)
            {
                s_tx_drop_cnt += (uint32_t)(len - i);
                break;
            }

            s_tx_buf[s_tx_head] = data[i];
            s_tx_head = next;
        }

        /* 启动DMA发送 */
        USART1_TxDmaKick();
    }
    taskEXIT_CRITICAL();

    return USART_OK;



    // uint16_t i;
    // USART_Status_t ret;

    // if(data == NULL || len == 0U)
    // {
    //     return USART_ERR_PARAM;
    // }

    // ret = USART1_Lock(USART1_MUTEX_TIMEOUT_MS);
    // if(ret != USART_OK)
    // {
    //     return ret;
    // }

    // for(i = 0U; i < len; ++i)
    // {
    //     USART1_SendRawByte(data[i]);
    // }

    // USART1_Unlock();
    // return USART_OK;
}

/* =========================
* 兼容旧接口
* ========================= */
void USART1_SendByte(const int8_t *Data, uint8_t len)
{
    if (Data == NULL || len == 0U)
        return;

    (void)USART1_SendBuffer((const uint8_t *)Data, (uint16_t)len);
}

/* =========================
 * 接收单字节
 * ========================= */
BaseType_t USART1_ReadByte(uint8_t *ch, TickType_t timeout_ticks)
{
    if(ch == NULL || s_usart_inited == 0U || s_rx_queue == NULL)
    {    
        return pdFALSE;
    }
    
    if(xQueueReceive(s_rx_queue, ch, timeout_ticks) == pdTRUE)
    {
        return pdTRUE;
    }
    return pdFALSE;
}


/* =========================
 * 接收缓冲区
 * ========================= */
uint16_t USART1_ReadBuffer(uint8_t *buf, uint16_t len, TickType_t timeout_ticks)
{
    uint16_t i;

    if(buf == NULL || len == 0U || s_usart_inited == 0U || s_rx_queue == NULL)
    {
        return 0U;
    }

    for(i = 0U; i < len; ++i)
    {
        if(xQueueReceive(s_rx_queue, &buf[i], timeout_ticks) != pdTRUE)
        {
            // 从此i开始，后面的数据全都不要了
            break;
        }
    }
    return i;  // 返回最后的索引
}


/* =========================
 * RX 缓冲状态
 * ========================= */
uint16_t USART1_RxPending(void)
{
    if(s_rx_queue == NULL)
    {
        return 0U;
    }

    return (uint16_t)uxQueueMessagesWaiting(s_rx_queue);  // 返回当前的索引
}

void USART1_ClearRxBuffer(void)
{
    if(s_rx_queue != NULL)
    {
        xQueueReset(s_rx_queue);
    }
}

uint32_t USART1_GetRxOverflowCount(void)
{
    return s_rx_overflow_cnt;
}
/* =========================
 * fputc 重定向
 * ========================= */
int fputc(int ch, FILE *f)
{
    (void)f;
    (void)USART1_SendChar((uint8_t)ch);
    return ch;
}

/* =========================
 * USART1 中断
 * ========================= */
void USART1_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t ch;
    uint16_t sr;

    sr = USART1->SR;

    if (sr & USART_SR_RXNE)
    {
        ch = (uint8_t)USART1->DR;   // 读DR清RXNE

        if (s_rx_queue != NULL)
        {
            if (xQueueSendFromISR(s_rx_queue, &ch, &xHigherPriorityTaskWoken) != pdTRUE)
            {
                s_rx_overflow_cnt++;
            }
        }
    }

    if (sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE))
    {
        volatile uint8_t dummy;
        dummy = (uint8_t)USART1->DR;  // 读DR清错误相关状态
        (void)dummy;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void DMA1_Channel4_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC4) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_TC4);

        /* 这一段已经发送完 */
        s_tx_tail = (uint16_t)((s_tx_tail + s_tx_dma_len) & USART1_TX_BUF_MASK);
        s_tx_dma_busy = 0U;

        /* 如果还有数据，立即启动下一段 */
        USART1_TxDmaKick();
    }

    if (DMA_GetITStatus(DMA1_IT_TE4) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_TE4);

        /* 出错时关闭 DMA，重置后尝试继续发送 */
        DMA_Cmd(DMA1_Channel4, DISABLE);
        s_tx_dma_busy = 0U;
        USART1_TxDmaKick();
    }
}