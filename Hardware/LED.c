#include "led.h"



/* ===================== GPIO 操作 ===================== */
/* 前灯：PB0, PA7  低电平亮 */
#define fLED_ON()         		\
do				  				\
{				  				\
	GPIOB->BRR = GPIO_Pin_0;	\
	GPIOA->BRR = GPIO_Pin_7;	\
}while(0)						
// (GPIOB->BRR  = GPIO_Pin_0 | GPIO_Pin_1)
#define fLED_OFF()				\
do				  				\
{				  				\
	GPIOB->BSRR = GPIO_Pin_0;	\
	GPIOA->BSRR = GPIO_Pin_7;	\
}while(0)						

#define fLED_Toggle()			\
do				  				\
{				  				\
	GPIOB->ODR ^= GPIO_Pin_0;	\
	GPIOA->ODR ^= GPIO_Pin_7;	\
}while(0)						

/* 后灯：PB1, PA6 低电平亮 */
#define bLED_ON()				\
do				  				\
{				  				\
	GPIOB->BRR = GPIO_Pin_1;	\
	GPIOA->BRR = GPIO_Pin_6;	\
}while(0)						

#define bLED_OFF() 				\
do				  				\
{				  				\
	GPIOB->BSRR = GPIO_Pin_1;	\
	GPIOA->BSRR = GPIO_Pin_6;	\
}while(0)						

#define bLED_Toggle()  			\
do				  				\
{				  				\
	GPIOB->ODR ^= GPIO_Pin_1;	\
	GPIOA->ODR ^= GPIO_Pin_6;	\
}while(0)						

/* ===================== 全局变量 ===================== */
/* 默认 100ms 闪烁，状态为全灯闪烁 */
volatile LED_s s_led = {.FlashTime = 200U, .status = AllFlashLight};



/* ===================== 软件呼吸参数 ===================== */



/* 任务周期：LED 不需要 1ms 刷新，10ms 更适合低负载运行 */
#define LED_TASK_PERIOD_MS			1U

/* ===================== 静态变量 ===================== */
static TaskHandle_t s_led_task_handle = NULL;

static TickType_t s_last_flash_tick = 0;
static TickType_t s_last_breath_tick = 0;


static uint8_t s_alt_phase = 0;		/* 交替闪烁相位 */
static uint8_t s_led_initialized = 0U;



#define BREATH_ENV_STEP_MS     10U      /* 呼吸亮度每隔多少 ms 调整一次 */
#define BREATH_ENV_MAX         100U     /* 呼吸亮度包络最大值：0~100 */
#define PWM_FRAME_TICKS        10U      /* 软件 PWM 周期：10ms -> 100Hz */

/* 呼吸灯状态变量 */
static uint8_t  s_pwm_phase = 0U;       /* 0~9 */
static uint8_t  s_pwm_duty = 0U;        /* 0~10，对应 PWM_FRAME_TICKS */
static uint8_t  s_breath_level = 0U;    /* 0~100 呼吸包络 */
static int8_t   s_breath_dir = 1;       /* 1 变亮，-1 变暗 */
static uint8_t  s_last_output = 0xFFU;  /* 记录上一次输出状态，减少重复写 GPIO */

/* ===================== 内部工具函数 ===================== */
static uint8_t LED_BreathDutyFromLevel(uint8_t level)
{
    /* level: 0~100
       duty : 0~10
       用平方近似做亮度修正 */
    uint16_t x = (uint16_t)level;
    uint16_t y = (uint16_t)((x * x + 50U) / 100U);  /* 0~100 */
    uint8_t duty = (uint8_t)((y * PWM_FRAME_TICKS + (BREATH_ENV_MAX / 2U)) / BREATH_ENV_MAX);

    if (duty > PWM_FRAME_TICKS)
    {
        duty = PWM_FRAME_TICKS;
    }

    return duty;
}

static TickType_t LED_GetNowTick(void)
{
	if(xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
	{
		return xTaskGetTickCount();
	}
	return 0;
}

static void LED_ResetInternalState(void)
{
	TickType_t now = LED_GetNowTick();

	taskENTER_CRITICAL();
	s_last_breath_tick = now;
	s_last_flash_tick = now;

	s_pwm_phase = 0U;
    s_pwm_duty = 0U;
    s_breath_level = 0U;
    s_breath_dir = 1;
    s_alt_phase = 0U;
    s_last_output = 0xFFU;   /* 强制下次刷新输出 */
	taskEXIT_CRITICAL();
}

/* 前灯/后灯操作 */
static void LED_FrontOn(void)
{
	fLED_ON();
}
static void LED_FrontOff(void)
{
	fLED_OFF();
}
static void LED_FrontToggle(void)
{
	fLED_Toggle();
}
static void LED_BackOn()
{
	bLED_ON();
}
static void LED_BackOff()
{
	bLED_OFF();
}
static void LED_BackToggle(void)
{
	bLED_Toggle();
}

static void LED_AllOn(void)
{
	LED_FrontOn();
	LED_BackOn();
}
static void LED_AllOff(void)
{
	LED_FrontOff();
	LED_BackOff();
}

static void LED_AllToggle(void)
{
	LED_FrontToggle();
	LED_BackToggle();
}

LED_status_e LED_GetLedStatus(void);



static void LED_Breathing(const TickType_t now)
{
    /* 先更新呼吸包络：每 10ms 调整一次亮度 */
    if ((now - s_last_breath_tick) >= pdMS_TO_TICKS(BREATH_ENV_STEP_MS))
    {
        s_last_breath_tick = now;

        if (s_breath_dir > 0)
        {
            if (s_breath_level >= BREATH_ENV_MAX)
            {
                s_breath_level = BREATH_ENV_MAX;
                s_breath_dir = -1;
            }
            else
            {
                s_breath_level++;
            }
        }
        else
        {
            if (s_breath_level == 0U)
            {
                s_breath_dir = 1;
            }
            else
            {
                s_breath_level--;
            }
        }

        s_pwm_duty = LED_BreathDutyFromLevel(s_breath_level);
    }

    /* 软件 PWM：10ms 一个周期，100Hz */
    s_pwm_phase++;
    if (s_pwm_phase >= PWM_FRAME_TICKS)
    {
        s_pwm_phase = 0U;
    }

    /* 当前是否应该点亮 */
    if (s_pwm_phase < s_pwm_duty)
    {
        if (s_last_output != 1U)
        {
            LED_AllOn();
            s_last_output = 1U;
        }
    }
    else
    {
        if (s_last_output != 0U)
        {
            LED_AllOff();
            s_last_output = 0U;
        }
    }
}


void LED_Init(void)
{
	GPIO_InitTypeDef gpio_init_structure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

	gpio_init_structure.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init_structure.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_init_structure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;

	GPIO_Init(GPIOA, &gpio_init_structure);

	gpio_init_structure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;

	GPIO_Init(GPIOB, &gpio_init_structure);

	s_led_initialized = 1U;
	/* 默认关灯 */
	LED_AllOff();
}


/* ======================= 1ms LED服务函数========================*/
/**
 * @brief: 这个函数适合被freeRTOS任务周期调用，
 *
 */
static void LED_Service1ms(void)
{
	TickType_t now = LED_GetNowTick();
	LED_status_e led_state;
	led_state = LED_GetLedStatus();
	switch(led_state)
	{
		case AlwaysOff:
		{
			LED_AllOff();
			break;
		}

		case AlwaysOn:
		{
			LED_AllOn();
			break;
		}

		case AllFlashLight:
		{
			if((now - s_last_flash_tick) >= pdMS_TO_TICKS(s_led.FlashTime))
			{
				s_last_flash_tick = now;
				LED_AllToggle();
			}
			break;
		}

		case AlternateFlash:
		{
			if((now - s_last_flash_tick) >= pdMS_TO_TICKS(s_led.FlashTime))
			{
				 s_last_flash_tick = now;
				 s_alt_phase ^= 1U;

				 if(s_alt_phase)
				 {
					/*前灯亮，后灯灭 */
					LED_FrontOn();
					LED_BackOff();
				 }
				 else
				 {
					/* 前灯灭，后灯亮 */
					LED_FrontOff();
					LED_BackOn();
				 }

			}
			break;
		} 

		case WARNING:
		{
			/* 警告灯： 后灯闪烁，前灯关闭 */
			if((now - s_last_flash_tick) >= pdMS_TO_TICKS(s_led.FlashTime))
			{
				s_last_flash_tick = now;
				LED_FrontOff();
				LED_BackToggle();
			}
			break;
		}

		case DANGEROURS:
		{
			/* 危险状态：前灯闪烁，后灯亮 */
			if((now - s_last_flash_tick) >= pdMS_TO_TICKS(s_led.FlashTime))
			{
				s_last_flash_tick = now;
				LED_FrontToggle();
				LED_BackOn();
			}
			break;
		}

		case GET_OFFSET:
		{
            /*
             * 这个状态你原代码里没实现，这里给一个合理默认：
             * 慢闪全灯，表示“正在校准/等待”
             * 如果你不想要这个效果，直接改成 LED_AllOff() 即可。
             */
            if ((now - s_last_flash_tick) >= pdMS_TO_TICKS(500U))
            {
                s_last_flash_tick = now;
                LED_AllToggle();
            }
			break;
		}

		case BREATHING:
		{
			LED_Breathing(now);
			 break;
		}

		default:
		{
			LED_AllOff();
		}
	}
}



/*==================FreeRTOS LED task==================*/
void LED_Task(void *pvParameters)
{
	/* 防止未初始化 */
	if(LED_GetIsInitialized() == 0U)
	{
		LED_Init();
	}

	TickType_t xLastWakeTime;
	(void)pvParameters;
	xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
		LED_Service1ms();
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(LED_TASK_PERIOD_MS));
	}
}

/* ===================== FreeRTOS 初始化接口 ===================== */
BaseType_t LED_RTOS_Init(UBaseType_t task_priority)
{
	if(LED_GetIsInitialized() == 0U)
	{
    	LED_Init();
	}

    LED_ResetInternalState();

    return xTaskCreate(
        LED_Task,
        "LED",
        128,              /* 栈大小：单位是 word，不是字节 */
        NULL,
        task_priority,
        &s_led_task_handle
    );
}
/* ===================== 兼容旧代码入口 ===================== */
void PilotLED(void)
{
    LED_Service1ms();
}


/* ===================== 状态/参数设置 ===================== */
uint8_t LED_GetIsInitialized(void)
{
	uint8_t ret;
	taskENTER_CRITICAL();
	ret = s_led_initialized;
	taskEXIT_CRITICAL();

	return (ret == 1U) ? 1U : 0U;
}

void LED_SetStatus(LED_status_e status)
{
	taskENTER_CRITICAL();
	s_led.status = status;
	LED_ResetInternalState();
	taskEXIT_CRITICAL();
}

void LED_SetFlashTime(uint16_t flash_time)
{
	if(flash_time == 0U)
	{
		flash_time = 1U;
	}
	taskENTER_CRITICAL();
	s_led.FlashTime = flash_time;
	taskEXIT_CRITICAL();
}

LED_status_e LED_GetLedStatus(void)
{
	LED_status_e state;
	taskENTER_CRITICAL();
	state = s_led.status;
	taskEXIT_CRITICAL();
	return state;
}

uint16_t LED_GetFlashTime(void)
{
	uint16_t ret;
	taskENTER_CRITICAL();
	ret = s_led.FlashTime;
	taskEXIT_CRITICAL();

	return ret;
}