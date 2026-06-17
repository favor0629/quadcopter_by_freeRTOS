#include "Encoder.h"
#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "task.h"



/*==================宏定义=====================*/
/* ENCODER_LEFT_TIM 配置为左电机编码器模式 */
#define ENCODER_LEFT_TIM			TIM3
#define ENCODER_TIM3_RCC			RCC_APB1Periph_TIM3
#define ENCODER_TIM3_GPIO_RCC		RCC_APB2Periph_GPIOA
#define ENCODER_TIM3_PORT			GPIOA
#define ENCODER_TIM3_PINS			(GPIO_Pin_7 | GPIO_Pin_6)

/* TIM4 配置为右电机编码器模式 */
#define ENCODER_RIGHT_TIM			TIM4
#define ENCODER_TIM4_RCC			RCC_APB1Periph_TIM4
#define ENCODER_TIM4_PORT_RCC		RCC_APB2Periph_GPIOB
#define ENCODER_TIM4_PINS			(GPIO_Pin_7 | GPIO_Pin_6)
#define ENCODER_TIM4_PORT			GPIOB


/*==================内部函数=====================*/

/**
 * @brief 原子地读取并清除某个编码器定时器计数值，只能
 * 在任务上下文中调用，不可以在中断中调用
 */
static int16_t Encoder_ReadAndReset(TIM_TypeDef *TIMx)
{
    uint16_t count;

    taskENTER_CRITICAL(); // 进入临界区，禁止中断，确保原子操作
    count = (uint16_t)TIM_GetCounter(TIMx); // 读取当前计数值
    TIM_SetCounter(TIMx, 0); // 清零计数器
    taskEXIT_CRITICAL(); // 退出临界区，恢复中断

    return (int16_t)count; // 返回读取的计数值，转换为有符号类型
}



/******************外部函数定义***********************/
void Encoder_Init(void)
{
	/* ========== TIM3：左编码器 ========== */
	RCC_APB1PeriphClockCmd(ENCODER_TIM3_RCC, ENABLE);
	RCC_APB2PeriphClockCmd(ENCODER_TIM3_GPIO_RCC, ENABLE);

	GPIO_InitTypeDef gpio_init_structure;
	gpio_init_structure.GPIO_Mode = GPIO_Mode_IPU;
	gpio_init_structure.GPIO_Pin = ENCODER_TIM3_PINS;
	gpio_init_structure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ENCODER_TIM3_PORT, &gpio_init_structure);

	TIM_TimeBaseInitTypeDef tim_time_base_init_structure;
	TIM_TimeBaseStructInit(&tim_time_base_init_structure);
	tim_time_base_init_structure.TIM_ClockDivision = TIM_CKD_DIV1;
	tim_time_base_init_structure.TIM_CounterMode = TIM_CounterMode_Up;
	tim_time_base_init_structure.TIM_Period = 0xFFFF;
	tim_time_base_init_structure.TIM_Prescaler = 0;
	tim_time_base_init_structure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(ENCODER_LEFT_TIM, &tim_time_base_init_structure);

	TIM_ICInitTypeDef tim_ic_init_structure;
	TIM_ICStructInit(&tim_ic_init_structure);

	tim_ic_init_structure.TIM_Channel = TIM_Channel_1;
	tim_ic_init_structure.TIM_ICFilter = 0x0F;
	TIM_ICInit(ENCODER_LEFT_TIM, &tim_ic_init_structure);

	tim_ic_init_structure.TIM_Channel = TIM_Channel_2;
	tim_ic_init_structure.TIM_ICFilter = 0x0F;
	TIM_ICInit(ENCODER_LEFT_TIM, &tim_ic_init_structure);

	TIM_EncoderInterfaceConfig(ENCODER_LEFT_TIM,
	                           TIM_EncoderMode_TI12,
	                           TIM_ICPolarity_Rising,
	                           TIM_ICPolarity_Rising);

	TIM_SetCounter(ENCODER_LEFT_TIM, 0);
	TIM_Cmd(ENCODER_LEFT_TIM, ENABLE);

	/* ========== TIM4：右编码器 ========== */
	RCC_APB1PeriphClockCmd(ENCODER_TIM4_RCC, ENABLE);
	RCC_APB2PeriphClockCmd(ENCODER_TIM4_PORT_RCC, ENABLE);

	gpio_init_structure.GPIO_Mode = GPIO_Mode_IPU;
	gpio_init_structure.GPIO_Pin = ENCODER_TIM4_PINS;
	GPIO_Init(ENCODER_TIM4_PORT, &gpio_init_structure);

	TIM_TimeBaseStructInit(&tim_time_base_init_structure);
	tim_time_base_init_structure.TIM_ClockDivision = TIM_CKD_DIV1;
	tim_time_base_init_structure.TIM_CounterMode = TIM_CounterMode_Up;
	tim_time_base_init_structure.TIM_Period = 0xFFFF;
	tim_time_base_init_structure.TIM_Prescaler = 0;
	tim_time_base_init_structure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(ENCODER_RIGHT_TIM, &tim_time_base_init_structure);

	TIM_ICStructInit(&tim_ic_init_structure);

	tim_ic_init_structure.TIM_Channel = TIM_Channel_1;
	tim_ic_init_structure.TIM_ICFilter = 0x0F;
	TIM_ICInit(ENCODER_RIGHT_TIM, &tim_ic_init_structure);

	tim_ic_init_structure.TIM_Channel = TIM_Channel_2;
	tim_ic_init_structure.TIM_ICFilter = 0x0F;
	TIM_ICInit(ENCODER_RIGHT_TIM, &tim_ic_init_structure);

	TIM_EncoderInterfaceConfig(ENCODER_RIGHT_TIM,
	                           TIM_EncoderMode_TI12,
	                           TIM_ICPolarity_Rising,
	                           TIM_ICPolarity_Falling);

	TIM_SetCounter(ENCODER_RIGHT_TIM, 0);
	TIM_Cmd(ENCODER_RIGHT_TIM, ENABLE);
}


/**
  * @brief  获取编码器的增量值
  * @param  n 指定左电机还是右电机，范围：1（左电机），2（右电机）
  * @retval 自上次调用此函数后的编码器增量值
  * @note   读取并清零是原子操作，适合 FreeRTOS 多任务环境
  * @note   建议由唯一的控制任务周期调用，不要多个任务同时调用
  */
int16_t Encoder_Get(uint8_t n)
{
    if(n == ENCODER_LEFT)
    {
        return Encoder_ReadAndReset(ENCODER_LEFT_TIM);
    }
    else if(n == ENCODER_RIGHT)
    {
        return Encoder_ReadAndReset(ENCODER_RIGHT_TIM);
    }

    return 0; // 无效参数返回0
}
