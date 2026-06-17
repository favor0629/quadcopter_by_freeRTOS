#include "LED.h"
#include "stm32f10x_conf.h"

/*=======================宏定义======================*/
#define LED_RCC	     RCC_APB2Periph_GPIOC
#define LED_PORT	GPIOC
#define LED_PIN 	GPIO_Pin_13

// 呼吸灯参数
#define LED_BREATH_PWM_PERIOD   20U
#define LED_BREATH_STEP_MS     25U

// led任务周期，单位ms
#define LED_TASK_PERIOD_MS     1U

/*=======================全局变量======================*/
// 软件状态变量，由多个任务访问，需注意临界区保护
static volatile uint8_t s_led_output_on = 0; // 当前LED输出状态，0表示灭，1表示亮
static volatile uint8_t s_led_breath_enabled = 0; // 呼吸灯使能标志，0表示关闭，1表示开启
static volatile uint8_t s_led_breath_level = 0; // 呼吸亮度等级0-20
static volatile uint8_t s_led_breath_dir = 1; // 1:亮度增加，0：亮度减少
static volatile uint8_t s_pwm_count = 0; // PWM计数器
static volatile uint8_t s_step_count = 0; // 呼吸步进计数器

/*=======================内部函数定义======================*/
// 设置LED输出状态，1表示亮，0表示灭
static void LED_Write(uint8_t on)
{
	// 用此函数之前，需要初始化
	if(on)
	{
		GPIO_ResetBits(LED_PORT, LED_PIN); // 输出低电平点亮LED
	}
	else
	{
		GPIO_SetBits(LED_PORT, LED_PIN); // 输出高电平熄灭LED
	}
}

/*=======================外部函数定义======================*/
/**
 * @brief  LED初始化
 * @param  None
 */
void LED_Init(void)
{
	// 开启时钟
	RCC_APB2PeriphClockCmd(LED_RCC, ENABLE);
	// 配置GPIO
	GPIO_InitTypeDef gpio_init_structure;
	gpio_init_structure.GPIO_Mode = GPIO_Mode_Out_PP; // 推挽输出
	gpio_init_structure.GPIO_Pin = LED_PIN;
	gpio_init_structure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LED_PORT, &gpio_init_structure);
	
	// 初始参数，需要加临界区
	taskENTER_CRITICAL();
	s_led_output_on = 0;
	s_led_breath_enabled = 0;
	s_led_breath_dir = 1;
	s_led_breath_level = 0;
	s_pwm_count = 0;
	s_step_count = 0;
	LED_Write(0); // 初始状态灭

	taskEXIT_CRITICAL();

}


/**
 * @brief  LED点亮
 */
void LED_ON(void)
{
	taskENTER_CRITICAL();
	s_led_breath_enabled = 0;
	s_led_breath_level = 0;
	s_led_breath_dir = 1;
	s_pwm_count = 0;
	s_step_count = 0;
	LED_Write(1);
	taskEXIT_CRITICAL();
}

/**
 * @brief  LED熄灭
 */
void LED_OFF(void)
{
	taskENTER_CRITICAL();
	s_led_breath_enabled = 0;
	s_led_breath_level = 0;
	s_led_breath_dir = 1;
	s_pwm_count = 0;
	s_step_count = 0;
	LED_Write(0);
	taskEXIT_CRITICAL();
}

/**
 * @brief  LED切换状态
 */
void LED_Turn(void)
{
	taskENTER_CRITICAL();
	s_led_breath_enabled = 0;
	s_led_breath_level = 0;
	s_led_breath_dir = 1;
	s_pwm_count = 0;
	s_step_count = 0;

	if (s_led_output_on)
	{
		LED_Write(0);
	}
	else
	{
		LED_Write(1);
	}
	taskEXIT_CRITICAL();
}


/**
 * @brief 进入呼吸灯模式
 */
void LED_Breath(void)
{
	LED_BreathEnable(1);
}

void LED_BreathEnable(uint8_t enable)
{
	taskENTER_CRITICAL();
	
	s_led_breath_enabled = enable ? 1U : 0U;
	s_led_breath_level = 0;
	s_led_breath_dir = 1;
	s_pwm_count = 0;
	s_step_count = 0;

	if(s_led_breath_enabled == 0U)
	{
		LED_Write(0); // 关闭呼吸灯时，默认熄灭
	}

	taskEXIT_CRITICAL();  // 退出临界区
}

/**
 * @brief 呼吸灯状态机，这个函数必须已固定周期调用，例如每1ms调用一次
 */
void LED_BreathTick(void)
{
	taskENTER_CRITICAL();

	if(s_led_breath_enabled == 0U)
	{
		taskEXIT_CRITICAL();
		return; // 呼吸灯未启用，直接返回
	}
	s_step_count++;
	if(s_step_count >= LED_BREATH_STEP_MS)
	{
		s_step_count = 0;

		// 调整亮度等级
		if(s_led_breath_dir != 0U)
		{
			if(s_led_breath_level < LED_BREATH_PWM_PERIOD)
			{
				s_led_breath_level++;
			}
			else
			{
				s_led_breath_dir = 0U; // 达到最大亮度，切换为减小亮度
			}
		}
		else
		{
			if(s_led_breath_level > 0U)
			{
				s_led_breath_level--;
			}
			else
			{
				s_led_breath_dir = 1U; // 达到最小亮度，切换为增加亮度
			}
		}
	}
	// PWM控制LED亮灭
	if(s_pwm_count < s_led_breath_level)
	{
		LED_Write(1); // 亮
	}
	else
	{
		LED_Write(0); // 灭
	}

	s_pwm_count++;
	if(s_pwm_count >= LED_BREATH_PWM_PERIOD)
	{
		s_pwm_count = 0;
	}

	taskEXIT_CRITICAL();
}


/**
 * @brief FreeRTOS 下推荐使用这个任务周期驱动呼吸灯,如果系统中LED只是状态灯，也可以改低频率
 */
void LED_Task(void *argument)
{
	(void)argument;

	TickType_t last_wake_time = xTaskGetTickCount();
	LED_Breath();
	while(1)
	{
		LED_BreathTick();
		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(LED_TASK_PERIOD_MS));
	}
}

