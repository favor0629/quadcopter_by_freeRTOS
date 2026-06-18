#include "stm32f10x.h"
#include "Delay.h"

/**
 * @brief  初始化 TIM1 为 1MHz 计数器
 *         计数周期：1us
 * @note   必须在系统时钟配置完成后调用
 */
void Delay_Init(void)
{
    TIM_TimeBaseInitTypeDef tim_init;
    RCC_ClocksTypeDef clocks;
    uint32_t tim_clk;
    uint16_t psc;

    /* 开启 TIM1 时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    /* 获取当前系统时钟信息 */
    RCC_GetClocksFreq(&clocks);

    /*
     * TIM1 挂在 APB2 上：
     * - 若 APB2 分频 = 1，则 TIM1 时钟 = PCLK2
     * - 若 APB2 分频 != 1，则 TIM1 时钟 = 2 * PCLK2
     */
    tim_clk = clocks.PCLK2_Frequency;
    if (clocks.HCLK_Frequency != clocks.PCLK2_Frequency)
    {
        tim_clk *= 2U;
    }

    /*
     * 配成 1MHz 计数频率，即 1us 计数一次
     * psc = tim_clk / 1,000,000 - 1
     */
    psc = (uint16_t)(tim_clk / 1000000U - 1U);

    tim_init.TIM_Prescaler = psc;
    tim_init.TIM_CounterMode = TIM_CounterMode_Up;
    tim_init.TIM_Period = 0xFFFF;
    tim_init.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_init.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM1, &tim_init);

    /* 先清零计数器 */
    TIM_SetCounter(TIM1, 0);

    /* 启动 TIM1 */
    TIM_Cmd(TIM1, ENABLE);
}

/**
 * @brief  微秒级阻塞延时
 * @param  us 延时微秒数
 * @note   基于 TIM1 计数器，不占用 SysTick
 */
void Delay_us(uint32_t us)
{
    uint16_t start;
    uint16_t now;
    uint16_t wait;

    while (us > 0U)
    {
        /*
         * TIM1 是 16 位计数器，单次最大安全等待 65535us
         * 这里分段处理，避免超范围
         */
        if (us > 0xFFFFU)
        {
            wait = 0xFFFFU;
        }
        else
        {
            wait = (uint16_t)us;
        }

        start = TIM_GetCounter(TIM1);

        while (1)
        {
            now = TIM_GetCounter(TIM1);

            /* 16位无符号减法，天然支持计数回绕 */
            if ((uint16_t)(now - start) >= wait)
            {
                break;
            }
        }

        us -= wait;
    }
}

/**
 * @brief  毫秒级阻塞延时
 * @param  ms 延时毫秒数
 */
void Delay_ms(uint32_t ms)
{
    while (ms > 0U)
    {
        /*
         * 这里分段避免 Delay_us(ms * 1000) 可能溢出
         * 65ms 一段比较稳妥
         */
        if (ms >= 65U)
        {
            Delay_us(65000U);
            ms -= 65U;
        }
        else
        {
            Delay_us(ms * 1000U);
            ms = 0U;
        }
    }
}

/**
 * @brief  秒级阻塞延时
 * @param  s 延时时间（秒）
 */
void Delay_s(uint32_t s)
{
    while (s--)
    {
        Delay_ms(1000U);
    }
}