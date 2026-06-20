#include "nrf24l01.h"
#include "spi.h"
//#include "stm32f10x.h"
#include "misc.h"
#include "string.h"





/* ============================================================
 * 你的地址配置
 * 注意：发送端和接收端必须一致或按你的协议匹配
 * ============================================================ */
static const uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0xE1, 0xE2, 0xE3, 0xE4, 0xE5};
static const uint8_t RX_ADDRESS[RX_ADR_WIDTH] = {0xE1, 0xE2, 0xE3, 0xE4, 0xE5};


/* ============================================================
 * 引脚定义：PB3=CE, PB4=CSN, PB5=IRQ
 * ============================================================ */
#define NRF24L01_CSN_HIGH()     (GPIOB->BSRR = GPIO_Pin_4)
#define NRF24L01_CSN_LOW()      (GPIOB->BRR  = GPIO_Pin_4)

#define NRF24L01_CE_HIGH()      (GPIOB->BSRR = GPIO_Pin_3)
#define NRF24L01_CE_LOW()       (GPIOB->BRR  = GPIO_Pin_3)

/* IRQ 低电平有效 */
#define NRF24L01_IRQ_READ()     (((GPIOB->IDR & GPIO_Pin_5) == GPIO_Pin_5) ? 1U : 0U)


/* ============================================================
 * 内部状态
 * ============================================================ */
typedef enum
{
    NRF24L01_MODE_UNKNOWN = 0U,
    NRF24L01_MODE_RX      = 1U,
    NRF24L01_MODE_TX      = 2U
} NRF24L01_Mode_t;


static volatile TaskHandle_t s_waitingTask = NULL;
static volatile NRF24L01_Mode_t s_current_mode = NRF24L01_MODE_UNKNOWN;
static volatile uint8_t s_initialized = 0U;     // 是否初始化


/* ============================================================
 * 内部辅助：互斥锁
 * 这里使用的SPI的锁，所以要确保SPI正确可用
 * ============================================================ */
static BaseType_t prv_lock(uint32_t timeout_ms)
{
    return (SPI_Lock(timeout_ms) == SPI_OK) ? pdTRUE : pdFALSE;
}

static BaseType_t prv_unlock(void)
{
    //SPI_Unlock();
    return (SPI_Unlock() == SPI_OK) ? pdTRUE : pdFALSE;
}


/* ============================================================
 * 内部辅助：命令 / 读写（不加锁版本）
 * 前提：调用者已持有 SPI 总线锁，且 CSN 已经拉低
 * ============================================================ */
static uint8_t prv_cmd(uint8_t cmd)
{
    uint8_t rx;
    NRF24L01_CSN_LOW();
    rx = SPI_RW_Byte(cmd);
    NRF24L01_CSN_HIGH();
    return rx;
}

static uint8_t prv_write_reg(uint8_t regaddr, uint8_t data)
{
    NRF24L01_CSN_LOW();
    (void)SPI_RW_Byte(SPI_WRITE_REG | (regaddr & 0x1FU));
    //SPI_RW_Byte(regaddr);
    (void)SPI_RW_Byte(data);
    NRF24L01_CSN_HIGH();
    return NRF24L01_OK;
}

static uint8_t prv_read_reg(uint8_t regaddr, uint8_t *rx)
{
    NRF24L01_CSN_LOW();
    (void)SPI_RW_Byte(SPI_READ_REG | (regaddr & 0x1FU));
    //SPI_RW_Byte(regaddr);
    *rx = SPI_RW_Byte(NOP);
    NRF24L01_CSN_HIGH();
    return NRF24L01_OK;
}

static uint8_t prv_write_buf(uint8_t cmd, const uint8_t *pBuf, uint8_t datalen)
{
    uint8_t i;

    if(pBuf == NULL || datalen == 0U || datalen > 32U)
    {
        return NRF24L01_ERROR;
    }

    NRF24L01_CSN_LOW();
    (void)SPI_RW_Byte(cmd);
    for(i = 0U; i < datalen; ++i)
    {
        (void)SPI_RW_Byte(pBuf[i]);
    }
    NRF24L01_CSN_HIGH();

    return NRF24L01_OK;
}

static uint8_t prv_read_buf(uint8_t cmd, uint8_t *pBuf, uint8_t datalen)
{
    uint8_t i;

    if(pBuf == NULL || datalen == 0U || datalen > 32U)
    {
        return NRF24L01_ERROR;
    }

    NRF24L01_CSN_LOW();
    (void)SPI_RW_Byte(cmd);
    for(i = 0U; i < datalen; ++i)
    {
        pBuf[i] = SPI_RW_Byte(NOP);
    }
    NRF24L01_CSN_HIGH();

    return NRF24L01_OK;
}


static void prv_clear_irq_flags(uint8_t flags)
{
    (void)prv_write_reg(STATUS, flags);
}


static void prv_flush_tx(void)
{
    (void)prv_cmd(FLUSH_TX);
}


static void prv_flush_rx(void)
{
    (void)prv_cmd(FLUSH_RX);
}


/* ============================================================
 * 内部辅助：进入 RX/TX 模式（不加锁版本）
 * ============================================================ */
static void prv_set_rx_mode(void)
{
    NRF24L01_CE_LOW();

    prv_flush_rx();
    prv_clear_irq_flags(MAX_TX | TX_OK | RX_OK);

    (void)prv_write_buf(SPI_WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH);

    /* 关闭自动应答，保持与你原工程兼容 */
    (void)prv_write_reg(EN_AA, 0x00U);

    /* 只开通道 0 */
    (void)prv_write_reg(EN_RXADDR, 0x01U);

    /* 地址宽度 5 字节 */
    (void)prv_write_reg(SETUP_AW, 0x03U);

    /* 自动重发参数，保留原配置 */
    (void)prv_write_reg(SETUP_RETR, 0x1AU);

    /* 射频通道 */
    (void)prv_write_reg(RF_CH, 1U);

    /* 固定 32 字节 */
    (void)prv_write_reg(RX_PW_P0, RX_PLOAD_WIDTH);

    /* 射频参数，保留你原工程配置 */
    (void)prv_write_reg(RF_SETUP, 0x07U);

    /* CONFIG:
     * bit0 PRIM_RX = 1
     * bit1 PWR_UP   = 1
     * bit2 EN_CRC   = 1
     * bit3 CRCO     = 1
     */
    (void)prv_write_reg(CONFIG, 0x0FU);

    NRF24L01_CE_HIGH();

    s_current_mode = NRF24L01_MODE_RX;
}



static void prv_set_tx_mode(void)
{
    NRF24L01_CE_LOW();

    prv_flush_tx();
    prv_clear_irq_flags(MAX_TX | TX_OK | RX_OK);

    (void)prv_write_buf(SPI_WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);
    (void)prv_write_buf(SPI_WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH);

    /* 关闭自动应答，保持兼容 */
    (void)prv_write_reg(EN_AA, 0x00U);

    /* 只保留通道 0 */
    (void)prv_write_reg(EN_RXADDR, 0x01U);

    (void)prv_write_reg(SETUP_AW, 0x03U);
    (void)prv_write_reg(SETUP_RETR, 0x1AU);
    (void)prv_write_reg(RF_CH, 1U);
    (void)prv_write_reg(RF_SETUP, 0x07U);

    /* CONFIG:
     * bit0 PRIM_RX = 0
     * bit1 PWR_UP   = 1
     * bit2 EN_CRC   = 1
     * bit3 CRCO     = 1
     */
    (void)prv_write_reg(CONFIG, 0x0EU);

    NRF24L01_CE_LOW();

    s_current_mode = NRF24L01_MODE_TX;
}


/* ============================================================
 * GPIO + SPI 初始化
 * ============================================================ */
void NRF24L01_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* PB3 -> CE, PB4 -> CSN */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    NRF24L01_CE_LOW();
    NRF24L01_CSN_HIGH();

    /* PB5 -> IRQ，输入上拉 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    SPI_Config();
}


/* ============================================================
 * EXTI 配置：PB5 -> EXTI5，下降沿触发
 * 注意：NVIC 优先级要符合 FreeRTOS 配置
 * ============================================================ */
void NRF24L01_EXTI_Init(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);

    EXTI_ClearITPendingBit(EXTI_Line5);

    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* 这个优先级你要按 FreeRTOSConfig.h 调整。
       原则：不能高于 configMAX_SYSCALL_INTERRUPT_PRIORITY 允许范围。 */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


/* ============================================================
 * 模块检查
 * ============================================================ */
uint8_t NRF24L01_Check(void)
{
    uint8_t txbuf[5] = {0xA5U, 0xA5U, 0xA5U, 0xA5U, 0xA5U};
    uint8_t rxbuf[5];
    uint8_t i;

    if (prv_lock(20U) != pdTRUE)
    {
        return 1U;
    }

    (void)prv_write_buf(SPI_WRITE_REG + TX_ADDR, txbuf, 5U);
    (void)prv_read_buf(SPI_READ_REG + TX_ADDR, rxbuf, 5U);

    prv_unlock();

    for (i = 0U; i < 5U; i++)
    {
        if (rxbuf[i] != txbuf[i])
        {
            return 1U;
        }
    }

    return 0U;
}


/* ============================================================
 * 底层寄存器操作：对外接口（带互斥）
 * ============================================================ */
uint8_t NRF24L01_Write_Reg(uint8_t regaddr, uint8_t data)
{
    uint8_t status;

    if (prv_lock(20U) != pdTRUE)
    {
        return NRF24L01_ERROR;
    }

    status = prv_write_reg(regaddr, data);

    prv_unlock();
    return status;
}



uint8_t NRF24L01_Read_Reg(uint8_t regaddr, uint8_t *rx)
{
    uint8_t status;

    if (prv_lock(20U) != pdTRUE)
    {
        return NRF24L01_ERROR;
    }

    status = prv_read_reg(regaddr, rx);

    prv_unlock();
    return status;
}

uint8_t NRF24L01_Write_Buf(uint8_t regaddr, const uint8_t *pBuf, uint8_t datalen)
{
    uint8_t status;

    if (prv_lock(20U) != pdTRUE)
    {
        return NRF24L01_ERROR;
    }

    status = prv_write_buf(regaddr, pBuf, datalen);

    prv_unlock();
    return status;
}


uint8_t NRF24L01_Read_Buf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen)
{
    uint8_t status;

    if (prv_lock(20U) != pdTRUE)
    {
        return NRF24L01_ERROR;
    }

    status = prv_read_buf(regaddr, pBuf, datalen);

    prv_unlock();
    return status;
}



/* ============================================================
 * 模式切换：对外接口
 * ============================================================ */
void NRF24L01_RX_Mode(void)
{
    if (prv_lock(50U) != pdTRUE)
    {
        return;
    }

    prv_set_rx_mode();

    prv_unlock();
}

void NRF24L01_TX_Mode(void)
{
    if (prv_lock(50U) != pdTRUE)
    {
        return;
    }

    prv_set_tx_mode();

    prv_unlock();
}

/* ============================================================
 * 初始化
 * ============================================================ */
NRF24L01_Status_t NRF24L01_Init(void)
{
    uint8_t retry;
    BaseType_t ok = pdFALSE;

    NRF24L01_Configuration();
    NRF24L01_EXTI_Init();

    /* 给芯片上电后一点稳定时间 */
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    else
    {
        for (volatile uint32_t i = 0; i < 200000U; i++)
        {
            __NOP();
        }
    }

    /* 尝试多次，避免刚上电时误判 */
    for (retry = 0U; retry < 10U; retry++)
    {
        if (NRF24L01_Check() == 0U)
        {
            ok = pdTRUE;
            break;
        }

        if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
        {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        else
        {
            for (volatile uint32_t i = 0; i < 50000U; i++)
            {
                __NOP();
            }
        }
    }

    if (ok != pdTRUE)
    {
        NRF24L01_CE_LOW();
        NRF24L01_CSN_HIGH();
        s_initialized = 0U;
        s_current_mode = NRF24L01_MODE_UNKNOWN;
        return NRF24L01_ERROR;
    }

    NRF24L01_RX_Mode();
    s_initialized = 1U;

    return NRF24L01_OK;
}

/* ============================================================
 * 发送一个 32 字节数据包
 * FreeRTOS 版：阻塞等待 IRQ，带超时
 * ============================================================ */
NRF24L01_Status_t NRF24L01_TxPacket(uint8_t *txbuf)
{
    uint8_t state;
    NRF24L01_Status_t ret = NRF24L01_ERROR;

    if ((txbuf == NULL) || (s_initialized == 0U))
    {
        return NRF24L01_INVALID_PARAM;
    }

    if (prv_lock(100U) != pdTRUE)
    {
        return NRF24L01_BUSY;
    }

    if (s_current_mode != NRF24L01_MODE_TX)
    {
        prv_set_tx_mode();
    }

    /* 清标志、清 FIFO，避免上一次残留影响这次发送 */
    prv_clear_irq_flags(MAX_TX | TX_OK | RX_OK);
    prv_flush_tx();

    /* 写入 32 字节 */
    (void)prv_write_buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH);

    /* 记录当前等待任务，IRQ 到来时唤醒它 */
    taskENTER_CRITICAL();
    s_waitingTask = xTaskGetCurrentTaskHandle();
    taskEXIT_CRITICAL();

    /* 开始发送 */
    NRF24L01_CE_HIGH();

    /* 等待 IRQ 通知 */
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50)) == 0U)
    {
        /* 超时：收尾，防止卡死 */
        NRF24L01_CE_LOW();
        prv_flush_tx();
        prv_clear_irq_flags(MAX_TX | TX_OK | RX_OK);

        taskENTER_CRITICAL();
        s_waitingTask = NULL;
        taskEXIT_CRITICAL();

        prv_unlock();
        return NRF24L01_TIMEOUT;
    }

    /* 收到中断后停止发送 */
    NRF24L01_CE_LOW();

    /* 清掉等待任务指针 */
    taskENTER_CRITICAL();
    s_waitingTask = NULL;
    taskEXIT_CRITICAL();

    prv_read_reg(STATUS, &state);
    prv_clear_irq_flags(state & (MAX_TX | TX_OK | RX_OK));

    if ((state & MAX_TX) != 0U)
    {
        prv_flush_tx();
        ret = NRF24L01_MAX_RT;
    }
    else if ((state & TX_OK) != 0U)
    {
        ret = NRF24L01_OK;
    }
    else
    {
        ret = NRF24L01_ERROR;
    }

    prv_unlock();
    return ret;
}

/* ============================================================
 * 非阻塞接收：轮询一次
 * 说明：
 * 1) 如果 STATUS 里有 RX_OK，就读一包
 * 2) 如果 STATUS 没置位，但 FIFO 里确实还有数据，也顺便读一包
 * 3) 读完后如果 FIFO 里还有残留，清掉，避免 IRQ 一直挂着
 * ============================================================ */
NRF24L01_Status_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
    uint8_t state;
    uint8_t fifo;
    NRF24L01_Status_t ret = NRF24L01_ERROR;

    if ((rxbuf == NULL) || (s_initialized == 0U))
    {
        return NRF24L01_INVALID_PARAM;
    }

    if (prv_lock(20U) != pdTRUE)
    {
        return NRF24L01_BUSY;
    }

    prv_read_reg(STATUS, &state);
    prv_read_reg(FIFO_STATUS, &fifo);

    /* RX_OK 置位，或者 FIFO 非空，都认为有包可读 */
    if (((state & RX_OK) != 0U) || ((fifo & 0x01U) == 0U))
    {
        (void)prv_read_buf(RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH);
        prv_clear_irq_flags(RX_OK);

        /* 如果还有残余包，但当前接口只有一个缓冲区，就清掉剩余 FIFO */
        prv_read_reg(FIFO_STATUS, &fifo);
        if ((fifo & 0x01U) == 0U)
        {
            prv_flush_rx();
        }

        ret = NRF24L01_OK;
    }
    else
    {
        if ((state & MAX_TX) != 0U)
        {
            prv_clear_irq_flags(MAX_TX);
        }
        ret = NRF24L01_ERROR;
    }

    prv_unlock();
    return ret;
}

/* ============================================================
 * 阻塞接收：等待 IRQ 唤醒
 * timeout: FreeRTOS tick
 * ============================================================ */
NRF24L01_Status_t NRF24L01_WaitRxPacket(uint8_t *rxbuf, TickType_t timeout)
{
    uint8_t state;
    uint8_t fifo;
    NRF24L01_Status_t ret = NRF24L01_ERROR;

    if ((rxbuf == NULL) || (s_initialized == 0U))
    {
        return NRF24L01_INVALID_PARAM;
    }

    if (prv_lock(100U) != pdTRUE)
    {
        return NRF24L01_BUSY;
    }

    if (s_current_mode != NRF24L01_MODE_RX)
    {
        prv_set_rx_mode();
    }

    /* 如果当前已经有数据，直接读，不必等待 */
    prv_read_reg(STATUS, &state);
    prv_read_reg(FIFO_STATUS, &fifo);

    if (((state & RX_OK) != 0U) || ((fifo & 0x01U) == 0U))
    {
        (void)prv_read_buf(RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH);
        prv_clear_irq_flags(RX_OK);

        prv_read_reg(FIFO_STATUS, &fifo);
        if ((fifo & 0x01U) == 0U)
        {
            prv_flush_rx();
        }

        prv_unlock();
        return NRF24L01_OK;
    }

    /* 记录等待任务 */
    taskENTER_CRITICAL();
    s_waitingTask = xTaskGetCurrentTaskHandle();
    taskEXIT_CRITICAL();

    /* 等待 IRQ */
    if (ulTaskNotifyTake(pdTRUE, timeout) == 0U)
    {
        taskENTER_CRITICAL();
        s_waitingTask = NULL;
        taskEXIT_CRITICAL();

        prv_unlock();
        return NRF24L01_TIMEOUT;
    }

    taskENTER_CRITICAL();
    s_waitingTask = NULL;
    taskEXIT_CRITICAL();

    prv_read_reg(STATUS, &state);
    prv_read_reg(FIFO_STATUS, &fifo);

    if (((state & RX_OK) != 0U) || ((fifo & 0x01U) == 0U))
    {
        (void)prv_read_buf(RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH);
        prv_clear_irq_flags(RX_OK);

        prv_read_reg(FIFO_STATUS, &fifo);
        if ((fifo & 0x01U) == 0U)
        {
            prv_flush_rx();
        }

        ret = NRF24L01_OK;
    }
    else
    {
        if ((state & MAX_TX) != 0U)
        {
            prv_clear_irq_flags(MAX_TX);
        }
        ret = NRF24L01_ERROR;
    }

    prv_unlock();
    return ret;
}

/* ============================================================
 * EXTI ISR 里调用这个函数
 * 这里只负责唤醒等待任务，不在 ISR 里做 SPI 操作
 * ============================================================ */
void NRF24L01_IRQHandlerFromISR(BaseType_t *pxHigherPriorityTaskWoken)
{
    TaskHandle_t task = s_waitingTask;

    if (task != NULL)
    {
        vTaskNotifyGiveFromISR(task, pxHigherPriorityTaskWoken);
    }
}