#include "BlueSerial.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "stm32f10x_conf.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define BLUE_SERIAL_PORT_RCC       RCC_APB2Periph_GPIOA
#define BLUE_SERIAL_PORT           GPIOA
#define BLUE_SERIAL_USART_RCC      RCC_APB1Periph_USART2
#define BLUE_SERIAL_USART          USART2
#define BLUE_SERIAL_PIN_TX         GPIO_Pin_2
#define BLUE_SERIAL_PIN_RX         GPIO_Pin_3
#define BLUE_SERIAL_RX_PACKET_SIZE (sizeof(BlueSerial_RxPacket))

char BlueSerial_RxPacket[100] = {0};
volatile uint8_t BlueSerial_RxFlag = 0;

static char BlueSerial_RxTempPacket[100] = {0};
static SemaphoreHandle_t BlueSerial_TxMutex = NULL;

static void BlueSerial_LockTx(void)
{
    if (BlueSerial_TxMutex != NULL)
    {
        xSemaphoreTake(BlueSerial_TxMutex, portMAX_DELAY);
    }
}

static void BlueSerial_UnlockTx(void)
{
    if (BlueSerial_TxMutex != NULL)
    {
        xSemaphoreGive(BlueSerial_TxMutex);
    }
}

static void BlueSerial_SendByteRaw(uint8_t byte)
{
    USART_SendData(BLUE_SERIAL_USART, byte);
    while (USART_GetFlagStatus(BLUE_SERIAL_USART, USART_FLAG_TXE) == RESET)
    {
    }
}

void BlueSerial_Init(void)
{
    GPIO_InitTypeDef gpio_init_structure;
    USART_InitTypeDef usart_init_structure;
    NVIC_InitTypeDef nvic_init_structure;

    RCC_APB2PeriphClockCmd(BLUE_SERIAL_PORT_RCC, ENABLE);
    RCC_APB1PeriphClockCmd(BLUE_SERIAL_USART_RCC, ENABLE);

    gpio_init_structure.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_init_structure.GPIO_Pin = BLUE_SERIAL_PIN_TX;
    gpio_init_structure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BLUE_SERIAL_PORT, &gpio_init_structure);

    gpio_init_structure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    gpio_init_structure.GPIO_Pin = BLUE_SERIAL_PIN_RX;
    GPIO_Init(BLUE_SERIAL_PORT, &gpio_init_structure);

    USART_StructInit(&usart_init_structure);
    usart_init_structure.USART_BaudRate = 9600;
    usart_init_structure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init_structure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart_init_structure.USART_Parity = USART_Parity_No;
    usart_init_structure.USART_StopBits = USART_StopBits_1;
    usart_init_structure.USART_WordLength = USART_WordLength_8b;
    USART_Init(BLUE_SERIAL_USART, &usart_init_structure);

    USART_ITConfig(BLUE_SERIAL_USART, USART_IT_RXNE, ENABLE);

    nvic_init_structure.NVIC_IRQChannel = USART2_IRQn;
    nvic_init_structure.NVIC_IRQChannelCmd = ENABLE;
    nvic_init_structure.NVIC_IRQChannelPreemptionPriority = 5;
    nvic_init_structure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&nvic_init_structure);

#if (configUSE_MUTEXES == 1)
    BlueSerial_TxMutex = xSemaphoreCreateMutex();
#else
    BlueSerial_TxMutex = NULL;
#endif

    USART_Cmd(BLUE_SERIAL_USART, ENABLE);
}

void BlueSerial_SendByte(uint8_t byte)
{
    BlueSerial_LockTx();
    BlueSerial_SendByteRaw(byte);
    BlueSerial_UnlockTx();
}

void BlueSerial_SendArray(uint8_t *array, uint16_t length)
{
    uint16_t i;

    if (array == NULL || length == 0)
    {
        return;
    }

    BlueSerial_LockTx();
    for (i = 0; i < length; ++i)
    {
        BlueSerial_SendByteRaw(array[i]);
    }
    BlueSerial_UnlockTx();
}

void BlueSerial_SendString(char *string)
{
    if (string == NULL)
    {
        return;
    }

    BlueSerial_SendArray((uint8_t *)string, (uint16_t)strlen(string));
}

static uint32_t BlueSerial_Pow(uint32_t x, uint32_t y)
{
    uint32_t result = 1;

    while (y--)
    {
        result *= x;
    }

    return result;
}

void BlueSerial_SendNumber(uint32_t number, uint8_t length)
{
    uint8_t i;

    BlueSerial_LockTx();
    for (i = 0; i < length; i++)
    {
        BlueSerial_SendByteRaw((uint8_t)(number / BlueSerial_Pow(10, length - i - 1) % 10 + '0'));
    }
    BlueSerial_UnlockTx();
}

void BlueSerial_Printf(char *format, ...)
{
    char string[100];
    va_list args;

    if (format == NULL)
    {
        return;
    }

    va_start(args, format);
    vsnprintf(string, sizeof(string), format, args);
    va_end(args);

    BlueSerial_SendString(string);
}

uint8_t BlueSerial_GetRxFlag(void)
{
    uint8_t flag;

    taskENTER_CRITICAL();
    flag = BlueSerial_RxFlag;
    if (BlueSerial_RxFlag)
    {
        BlueSerial_RxFlag = 0;
    }
    taskEXIT_CRITICAL();

    return flag;
}

char *BlueSerial_GetRxPacket(void)
{
    return BlueSerial_RxPacket;
}

void USART2_IRQHandler(void)
{
    static uint8_t rx_state = 0;
    static uint8_t p_rx_packet = 0;

    if (USART_GetITStatus(BLUE_SERIAL_USART, USART_IT_RXNE) != RESET)
    {
        uint8_t rx_data = (uint8_t)USART_ReceiveData(BLUE_SERIAL_USART);

        if (rx_data == '[')
        {
            if (BlueSerial_RxFlag == 0)
            {
                rx_state = 1;
                p_rx_packet = 0;
            }
        }
        else if (rx_state == 1)
        {
            if (rx_data == ']')
            {
                uint8_t i;
                uint8_t copy_len = p_rx_packet;

                if (copy_len >= BLUE_SERIAL_RX_PACKET_SIZE)
                {
                    copy_len = BLUE_SERIAL_RX_PACKET_SIZE - 1;
                }

                for (i = 0; i < copy_len; ++i)
                {
                    BlueSerial_RxPacket[i] = BlueSerial_RxTempPacket[i];
                }

                BlueSerial_RxPacket[copy_len] = '\0';
                BlueSerial_RxFlag = 1;
                rx_state = 0;
                p_rx_packet = 0;
            }
            else if (p_rx_packet < (BLUE_SERIAL_RX_PACKET_SIZE - 1))
            {
                BlueSerial_RxTempPacket[p_rx_packet] = (char)rx_data;
                p_rx_packet++;
            }
            else
            {
                rx_state = 0;
                p_rx_packet = 0;
            }
        }
    }
}
