#include "spi.h"
#include "Delay.h"


/* =========================
 * 参数
 * ========================= */
#define SPI_MUTEX_TIMEOUT_MS     20U
#define SPI_BIT_DELAY_US         1U


/* =========================
 * 互斥锁
 * ========================= */
static StaticSemaphore_t s_spi_mutex_buf;
static SemaphoreHandle_t s_spi_mutex = NULL;
static uint8_t s_spi_inited = 0U;


/* =========================
 * 内部延时
 * ========================= */
static void SPI_BitDelay(void)
{
    Delay_us(SPI_BIT_DELAY_US);
}


/* =========================
 * 初始化
 * ========================= */

void SPI_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(SPI_GPIO_RCC, ENABLE);

    /* CS */
    GPIO_InitStructure.GPIO_Pin = SPI_CS_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(SPI_GPIO_PORT, &GPIO_InitStructure);

    /* SCK */
    GPIO_InitStructure.GPIO_Pin = SPI_SCK_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(SPI_GPIO_PORT, &GPIO_InitStructure);

    /* MOSI */
    GPIO_InitStructure.GPIO_Pin = SPI_MOSI_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(SPI_GPIO_PORT, &GPIO_InitStructure);

    /* MISO */
    GPIO_InitStructure.GPIO_Pin = SPI_MISO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(SPI_GPIO_PORT, &GPIO_InitStructure);

#if (configSUPPORT_STATIC_ALLOCATION == 1)
    if(s_spi_mutex == NULL)
        s_spi_mutex = xSemaphoreCreateMutexStatic(&s_spi_mutex_buf);
#else
    if(s_spi_mutex == NULL)
        s_spi_mutex = xSemaphoreCreateMutex();
#endif

    s_spi_inited = 1U;

    /* 空闲态 */
    SPI_CS_Deselect();
    SPI_SCK_L();
    SPI_MOSI_H();
}

SPI_Status_t SPI_Lock(TickType_t timeout_ms)
{
    if(s_spi_inited == 0U || s_spi_mutex == NULL)
    {
        return SPI_ERR_NOT_INIT;
    }

    if(xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
    {
        return SPI_ERR_TIMEOUT;
    }

    return SPI_OK;
}

SPI_Status_t SPI_Unlock(void)
{
    if(s_spi_inited == 0U || s_spi_mutex == NULL)
    {
        return SPI_ERR_NOT_INIT;
    }

    if(xSemaphoreGive(s_spi_mutex) != pdTRUE)
    {
        return SPI_ERR;
    }
    return SPI_OK;
}

/* =========================
 * CS 控制
 * ========================= */
void SPI_CS_Select(void)
{   
    SPI_CS_L();
    SPI_BitDelay();
}

void SPI_CS_Deselect(void)
{
    SPI_CS_H();
    SPI_BitDelay();
}


/* =========================
 * 单字节收发
 * 说明：
 *   这个函数是底层函数，
 *   要求调用前已经锁住总线，并且 CS 已经拉低。
 * ========================= */
uint8_t SPI_RW_Byte(uint8_t byte)
{
    uint8_t i;
    uint8_t rx = 0U;

    for(i = 0U; i < 8U; ++i)
    {
        SPI_SCK_L();
        SPI_BitDelay();

        if(byte & 0x80U)
        {
            SPI_MOSI_H();
        }
        else
        {
            SPI_MOSI_L();
        }

        byte <<= 1;
        SPI_BitDelay();

        SPI_SCK_H();
        SPI_BitDelay();

        rx <<= 1;
        if(SPI_MISO_READ())
        {
            rx |= 0x01U;
        }
        SPI_BitDelay();
    }

    SPI_SCK_L();
    SPI_BitDelay();

    return rx;
}


/* =========================
 * 单字节安全封装
 * ========================= */
SPI_Status_t SPI_ExchangeByte(uint8_t tx, uint8_t *rx)
{
    SPI_Status_t ret;

    if(rx == NULL)
    {
        return SPI_ERR_PARAM;
    }

    ret = SPI_Lock(SPI_MUTEX_TIMEOUT_MS);
    if(ret != SPI_OK)
    {
        return ret;
    }

    SPI_CS_Select();
    *rx = SPI_RW_Byte(tx);
    SPI_CS_Deselect();

    SPI_Unlock();

    return SPI_OK;
}
/* =========================
 * 多字节安全传输
 * ========================= */
SPI_Status_t SPI_Transfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    uint16_t i;
    uint8_t dummy_tx;
    SPI_Status_t ret;

    if(len == 0U)
    {
        return SPI_ERR_PARAM;
    }

    if(tx == NULL && rx == NULL)
    {
        return SPI_ERR_PARAM;
    }

    ret = SPI_Lock(SPI_MUTEX_TIMEOUT_MS);
    if(ret != SPI_OK)
    {
        return ret;
    }

    SPI_CS_Select();

    for(i = 0U; i < len; ++i)
    {
        dummy_tx = (tx != NULL)? tx[i] : 0xFFU;

        if(rx != NULL)
        {
            rx[i] = SPI_RW_Byte(dummy_tx);
        }
        else
        {
            (void)SPI_RW_Byte(dummy_tx);
        }
    }

    SPI_CS_Deselect();
    SPI_Unlock();

    return SPI_OK;
}

/* =========================
 * 只发
 * ========================= */
SPI_Status_t SPI_WriteBytes(const uint8_t *tx, uint16_t len)
{
    return SPI_Transfer(tx, NULL, len);
}


/* =========================
 * 只收
 * ========================= */
SPI_Status_t SPI_ReadBytes(uint8_t fill, uint8_t *rx, uint16_t len)
{
    uint16_t i;
    uint8_t tx_byte;
    SPI_Status_t ret;

    if (rx == NULL || len == 0U)
        return SPI_ERR_PARAM;

    ret = SPI_Lock(SPI_MUTEX_TIMEOUT_MS);
    if (ret != SPI_OK)
        return ret;

    SPI_CS_Select();

    tx_byte = fill;
    for (i = 0U; i < len; i++)
    {
        rx[i] = SPI_RW_Byte(tx_byte);
    }

    SPI_CS_Deselect();
    SPI_Unlock();

    return SPI_OK;
}
