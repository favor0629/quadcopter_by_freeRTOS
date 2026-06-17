#include "Timer.h"
#include "stm32f10x_conf.h"



/*==============================================================================
 * Public Function Prototypes
 *============================================================================*/


/**
 * @brief Initialize TIM3 update interrupt.
 *
 * @details
 * Configure TIM3 as a periodic timer and enable
 * its update interrupt.
 *
 * Timer Clock Source:
 * - APB1 Clock      : 36 MHz
 * - Timer Clock     : 72 MHz
 *   (APB1 prescaler is not 1, therefore timer clock
 *    is automatically doubled by hardware.)
 *
 * Timer Configuration:
 * - Prescaler       : 719
 * - Counter Clock   : 72 MHz / (719 + 1)
 *                    = 100 kHz
 *
 * - Auto Reload     : 299
 * - Update Period   : (299 + 1) / 100 kHz
 *                    = 3 ms
 *
 * Interrupt Configuration:
 * - IRQ Channel     : TIM3_IRQn
 * - Preemption Pri. : 1
 * - Sub Priority    : 3
 *
 * @note
 * This function enables both the TIM3 update interrupt
 * and the TIM3 peripheral.
 *
 * @retval None
 */
void TIM3_Config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  	NVIC_InitTypeDef NVIC_InitStructure; 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // enable clock
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3 interrupt
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //The IRQ channel is enabled
	
	/*
     * Configure TIM3 time base.
     *
     * Counter Frequency:
     *     72 MHz / (719 + 1)
     *   = 100 kHz
     *
     * Interrupt Period:
     *     (299 + 1) / 100 kHz
     *   = 3 ms
     */
	NVIC_Init(&NVIC_InitStructure);  
	TIM_TimeBaseStructure.TIM_Period = 299;  
	TIM_TimeBaseStructure.TIM_Prescaler =719; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM upward counting mode
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 
 
	/*Enable TIM3 update interrupt.*/
	TIM_ITConfig(  
		TIM3, //TIM3
		TIM_IT_Update ,
		ENABLE  
		);


	TIM_Cmd(TIM3, ENABLE);  // Start TIM3.			 
}




// // 开启定时时钟，采用的是TIM1
// void Timer_Init(void)
// {
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
// 	// 设置内部时钟源
// 	TIM_InternalClockConfig(TIM1);
	
// 	// 配置时基单元
// 	TIM_TimeBaseInitTypeDef tim_time_base_init_structure;
// 	tim_time_base_init_structure.TIM_ClockDivision = TIM_CKD_DIV1;
// 	tim_time_base_init_structure.TIM_CounterMode = TIM_CounterMode_Up;
// 	tim_time_base_init_structure.TIM_Period = 1000 - 1;
// 	tim_time_base_init_structure.TIM_Prescaler = 72 - 1;
// 	tim_time_base_init_structure.TIM_RepetitionCounter = 0;
// 	TIM_TimeBaseInit(TIM1, &tim_time_base_init_structure);
	
// 	// 中断输出配置
	
// 	TIM_ClearFlag(TIM1, TIM_FLAG_Update);	// 清除标志
	
// 	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	
// 	// 中断优先级分组
// 	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
// 	// 配置NVIC
// 	NVIC_InitTypeDef nvic_init_structure;
// 	nvic_init_structure.NVIC_IRQChannel = TIM1_UP_IRQn;
// 	nvic_init_structure.NVIC_IRQChannelCmd = ENABLE;
// 	nvic_init_structure.NVIC_IRQChannelPreemptionPriority = 6;
// 	nvic_init_structure.NVIC_IRQChannelSubPriority = 0;
// 	NVIC_Init(&nvic_init_structure);
	
// 	TIM_Cmd(TIM1, ENABLE);
// }



// /**********************************************************
// * @funcName ：TIM2_Init
// * @brief    ：对定时器2和定时器3进行初始化设置
// * @param    ：uint16_t arr (重载值)
// * @param    ：uint16_t psc (预分频值)
// * @retval   ：void
// * @details  ：
// * @fn       ：定时器2抢占优先级4，子优先级0；定时器3抢占优先级6，子优先级0；
// ************************************************************/

// void Timer2_Init(uint16_t arr, uint16_t psc)
// {
//     TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//     NVIC_InitTypeDef NVIC_InitStructure;

//     RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

//     TIM_TimeBaseStructure.TIM_Period = arr; // 设置重载值
//     TIM_TimeBaseStructure.TIM_Prescaler = psc; // 设置预分频值
//     TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//     TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//     TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
//     TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

//     NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
//     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4; // 抢占优先级4
//     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // 子优先级0
//     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//     NVIC_Init(&NVIC_InitStructure);

//     TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
//     TIM_Cmd(TIM2, ENABLE);

// }


// /**********************************************************
// * @funcName ：TIM3_Init
// * @brief    ：对定时器3进行初始化设置
// * @param    ：uint16_t arr (重载值)
// * @param    ：uint16_t psc (预分频值)
// * @retval   ：void
// * @details  ：
// * @fn       ：定时器3抢占优先级6，子优先级0；
// ************************************************************/
// void Timer3_Init(uint16_t arr, uint16_t psc)
// {
//     TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//     NVIC_InitTypeDef NVIC_InitStructure;

//     RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

//     TIM_TimeBaseStructure.TIM_Period = arr; // 设置重载值
//     TIM_TimeBaseStructure.TIM_Prescaler = psc; // 设置预分频值
//     TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//     TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//     TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
//     TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

//     NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
//     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6; // 抢占优先级6
//     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // 子优先级0
//     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//     NVIC_Init(&NVIC_InitStructure);

//     TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
//     TIM_Cmd(TIM3, ENABLE);
// }






// /**********************************************************
// * @funcName ：TIM2_IRQHandler
// * @brief    ：定时器2的中断服务函数
// * @param    ：void
// * @retval   ：void
// * @details  ：
// * @fn       ：
// ************************************************************/
// void TIM2_IRQHandler(void)
// {
//     if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
//     {
//         TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
//         //usart_printf("Timer 2 interrupt\r\n");
//         if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0)
//         {
//         GPIO_SetBits(GPIOA, GPIO_Pin_0);
//         }
//         else
//         {
//         GPIO_ResetBits(GPIOA, GPIO_Pin_0);
// 		}
//     }
// }

// /**********************************************************
// * @funcName ：TIM3_IRQHandler
// * @brief    ：定时器3的中断服务函数
// * @param    ：void
// * @retval   ：void
// * @details  ：
// * @fn       ：
// ************************************************************/
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        //usart_printf("Timer 3 interrupt\r\n");
        // if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == 0)
        // {
        // GPIO_SetBits(GPIOA, GPIO_Pin_1);
        // }
        // else
        // {
        // GPIO_ResetBits(GPIOA, GPIO_Pin_1);
		// }
    }
}