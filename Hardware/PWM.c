#include "pwm.h"
#include "FreeRTOS.h"
#include "task.h"

/***************************宏定义******************************** */
/*==============================================================================
 * Private Macros
 *==============================================================================*/
#define PWM_TIM TIM2
#define PWM_TIM_RCC RCC_APB1Periph_TIM2
#define PWM_PORT_RCC RCC_APB2Periph_GPIOA
#define PWM_PORT GPIOA
#define PWM_PINS (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3)



/*==============================================================================
 * Public Functions
 *==============================================================================*/
/**
 * @brief Initialize PWM outputs based on TIM2.
 *
 * @details
 * Configure TIM2 channels 1~4 as PWM outputs.
 *
 * Pin Mapping:
 * - PA0 -> TIM2_CH1
 * - PA1 -> TIM2_CH2
 * - PA2 -> TIM2_CH3
 * - PA3 -> TIM2_CH4
 *
 * Timer Configuration:
 * - Timer Clock      : 72 MHz
 * - Prescaler        : 8
 * - Counter Clock    : 72 MHz / (8 + 1) = 8 MHz
 * - Auto Reload      : 999
 * - PWM Frequency    : 8 MHz / 1000 = 8 kHz
 *
 * PWM Resolution:
 * - 1000 Levels (0 ~ 999)
 *
 * @note
 * System clock configuration must be completed
 * before calling this function.
 */
void PWM_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIO clock. */
	RCC_APB2PeriphClockCmd(PWM_PORT_RCC, ENABLE);

	/*Configure PWM output pins.*/
	GPIO_InitStructure.GPIO_Pin = PWM_PINS;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(PWM_PORT, &GPIO_InitStructure);
	/* Enable TIM2 clock */
	RCC_APB1PeriphClockCmd(PWM_TIM_RCC, ENABLE);
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 999; 
	TIM_TimeBaseStructure.TIM_Prescaler = 8; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 

	TIM_TimeBaseInit(PWM_TIM, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //Configure to PWM1 mode
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	/**
	 * Set the jump value. When the counter counts to this value, 
	 * the level jumps (i.e., the duty cycle). The initial value is 0
	 */
	TIM_OCInitStructure.TIM_Pulse = 0;
	 /**
	  * When the count of the timer is less than 
	  * the timing set value, it is at a high level
	  */
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	/* enable channel1 */
	TIM_OC1Init(PWM_TIM, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);
	/* enable channel2 */
	TIM_OC2Init(PWM_TIM, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);
	/* enable channel3 */
	TIM_OC3Init(PWM_TIM, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);
	/* enable channel4 */
	TIM_OC4Init(PWM_TIM, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(PWM_TIM, ENABLE); // enable overload register
	TIM_Cmd(PWM_TIM, ENABLE); //enable TIM2
}

/**
 * @brief Set PWM duty cycle for Channel 1.
 *
 * @param[in] compare
 *            Compare value.
 *
 * Duty Cycle:
 * Duty = compare / 1000
 *
 * Valid Range:
 * - 0   : 0%
 * - 500 : 50%
 * - 999 : 99.9%
 *
 * @note
 * The compare value should not exceed
 * the configured auto-reload value (999).
 */
void PWM_SetCompare1(uint16_t compare)
{
    taskENTER_CRITICAL();
    TIM_SetCompare1(PWM_TIM, compare);
    taskEXIT_CRITICAL();
}
void PWM_SetCompare2(uint16_t compare)
{
    taskENTER_CRITICAL();
    TIM_SetCompare2(PWM_TIM, compare);
    taskEXIT_CRITICAL();
}
void PWM_SetCompare3(uint16_t compare)
{
    taskENTER_CRITICAL();
    TIM_SetCompare3(PWM_TIM, compare);
    taskEXIT_CRITICAL();
}
void PWM_SetCompare4(uint16_t compare)
{
    taskENTER_CRITICAL();
    TIM_SetCompare4(PWM_TIM, compare);
    taskEXIT_CRITICAL();
}

