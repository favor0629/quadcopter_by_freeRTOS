#include "Key.h"
#include "usart.h"

#define KEY_RCC     (RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB)
#define KEY_PORT_2  GPIOA
#define KEY_PORT_1  GPIOB
#define KEY_PIN_22  GPIO_Pin_4
#define KEY_PIN_21  GPIO_Pin_5
#define KEY_PIN_11  GPIO_Pin_0
#define KEY_PIN_12  GPIO_Pin_1

volatile uint8_t g_key_number = 0;

static uint8_t Key_GetState(void);

void Key_Init(void)
{
    GPIO_InitTypeDef gpio_init_structure;

    RCC_APB2PeriphClockCmd(KEY_RCC, ENABLE);

    gpio_init_structure.GPIO_Mode = GPIO_Mode_IPU;
    gpio_init_structure.GPIO_Speed = GPIO_Speed_50MHz;

    gpio_init_structure.GPIO_Pin = KEY_PIN_21 | KEY_PIN_22;
    GPIO_Init(KEY_PORT_2, &gpio_init_structure);

    gpio_init_structure.GPIO_Pin = KEY_PIN_11 | KEY_PIN_12;
    GPIO_Init(KEY_PORT_1, &gpio_init_structure);
}

uint8_t Key_GetNum(void)
{
    uint8_t temp = 0;

    taskENTER_CRITICAL();
    if (g_key_number != 0)
    {
        temp = g_key_number;
        g_key_number = 0;
    }
    taskEXIT_CRITICAL();

    return temp;
}

static uint8_t Key_GetState(void)
{
    if (GPIO_ReadInputDataBit(KEY_PORT_1, KEY_PIN_12) == 0)
    {
        return 1;
    }
    if (GPIO_ReadInputDataBit(KEY_PORT_1, KEY_PIN_11) == 0)
    {
        return 2;
    }
    if (GPIO_ReadInputDataBit(KEY_PORT_2, KEY_PIN_21) == 0)
    {
        return 3;
    }
    if (GPIO_ReadInputDataBit(KEY_PORT_2, KEY_PIN_22) == 0)
    {
        return 4;
    }

    return 0;
}

void Key_Tick(void)
{
    static uint8_t last_sample = 0;
    static uint8_t stable_state = 0;
    static uint8_t stable_count = 0;

    uint8_t sample = Key_GetState();

    if (sample == last_sample)
    {
        if (stable_count < 20)
        {
            stable_count++;
        }
    }
    else
    {
        last_sample = sample;
        stable_count = 0;
    }

    if (stable_count >= 20 && sample != stable_state)
    {
        stable_state = sample;

        /* Generate one event when a key becomes stably pressed. */
        if (stable_state != 0)
        {
            taskENTER_CRITICAL();
            if (g_key_number == 0)
            {
                g_key_number = stable_state;
            }
            taskEXIT_CRITICAL();
        }
    }
}

void Key_Task(void *argument)
{
    TickType_t last_wake_time;

    (void)argument;

    //usart_printf("start Key task successfully\r\n");

    last_wake_time = xTaskGetTickCount();

    for (;;)
    {
        Key_Tick();
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
    }
}
