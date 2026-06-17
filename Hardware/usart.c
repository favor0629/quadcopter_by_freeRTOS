#include "usart.h"
#include "misc.h"
#include "stdio.h"
#include "task.h"

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
static uint32_t s_baudrate = 9600U;




/* =========================
 * 内部工具
 * ========================= */
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

    /* 先启用基本的 UART 硬件，即使互斥量创建失败也能进行裸发送 */
    s_baudrate = baudRate;
    USART1_ApplyConfig(s_baudrate);

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
    USART_Status_t ret;

    if(data == NULL || len == 0U)
    {
        return USART_ERR_PARAM;
    }

    ret = USART1_Lock(USART1_MUTEX_TIMEOUT_MS);
    if(ret != USART_OK)
    {
        return ret;
    }

    for(i = 0U; i < len; ++i)
    {
        USART1_SendRawByte(data[i]);
    }

    USART1_Unlock();
    return USART_OK;
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