/*******************************************************************
 *@brief 
 *@brief 
 *@time ZIN电子产品与技术 2017.1.8
 *@editor小南&zin
 *飞控爱好QQ群551883670,邮箱759421287@qq.com
 *非授权使用人员，禁止使用。禁止传阅，违者一经发现，侵权处理。
 *专业的飞控才是最好的飞控
 ******************************************************************/
//#include "stm32f10x.h"
#include "key.h"
// #include "remote.h"
// #include "sys.h"


/******************************************************************************
函数原型：
功    能：	按键初始化
*******************************************************************************/ 

//PA15 AUX1
//PB11 AUX2
//PC14 AUX3
//PC15 AUX4
//PB1 front
//PB0 back
//PA6 left
//PA7 right
void key_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	AFIO->MAPR = 0X02000000; //使用四线SWD
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC , ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_4|GPIO_Pin_15;      
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    //上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
}




/******************************************************************************
函数原型：
功    能：	按键扫描
*******************************************************************************/ 
uint8_t key(void)
{	
    volatile static uint8_t status = 0;	
 	static uint32_t temp;
    temp = get_SysTick_count();
	switch(status)
	{
		case 0:
			if(get_SysTick_count() - temp >30) //300ms 按键音
			{
				if(  ((GPIOA->IDR & (GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_15)) == (GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_15))
					&& ((GPIOB->IDR & (GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_11)) == (GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_11))
					&& ((GPIOC->IDR & (GPIO_Pin_14|GPIO_Pin_15)) == (GPIO_Pin_14|GPIO_Pin_15))
					)
					status = 1;
				//BEEP_L();
			}
			break;
		case 1:
			if(((GPIOA->IDR & (GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_15)) != (GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_15))
				|| ((GPIOB->IDR & (GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_11)) != (GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_11))
				|| ((GPIOC->IDR & (GPIO_Pin_14|GPIO_Pin_15)) != (GPIO_Pin_14|GPIO_Pin_15))
				)
				status = 2;
			break;

        default:
            break;
		// case 2:	
		// 	if(!(GPIOA->IDR & aux1))
		// 	{
		// 			Remote.AUX1 ^= (2000^1000); //1000和2000之间变化
		// 	}
		// 	else if(!(GPIOB->IDR & aux2))
		// 	{
		// 			Remote.AUX2 ^= (2000^1000); //1000和2000之间变化
		// 	}
		// 	else if(!(GPIOC->IDR & aux3))
		// 	{
		// 			Remote.AUX3 ^= (2000^1000); //1000和2000之间变化
		// 	}
		// 	else if(!(GPIOC->IDR & aux4))
		// 	{
		// 			Remote.AUX4 ^= (2000^1000); //1000和2000之间变化	
		// 	}			
		// 	else if(!(GPIOA->IDR & left))
		// 	{
		// 		if(offset.roll>-250)
		// 		{
		// 			offset.roll-=5;	
		// 			write_AT24C02(ROLL_ADDR_OFFSET,offset.roll);	//记录微调校准值	
		// 		}					
		// 	}
		// 	else if(!(GPIOA->IDR & right))
		// 	{
		// 		if(offset.roll<250)
		// 		{
		// 			offset.roll+=5;
		// 			write_AT24C02(ROLL_ADDR_OFFSET,offset.roll);
		// 		}					
		// 	}
		// 	else if(!(GPIOB->IDR & front))
		// 	{
		// 		if(offset.pitch>-250)
		// 		{
		// 			offset.pitch-=5;	
		// 			write_AT24C02(PITCH_ADDR_OFFSET,offset.pitch);	//记录微调校准值	
		// 		}
		// 	}
		// 	else if(!(GPIOB->IDR & back))
		// 	{
		// 		if(offset.pitch<250)
		// 		{
		// 			offset.pitch+=5;
		// 			write_AT24C02(PITCH_ADDR_OFFSET,offset.pitch);	//记录微调校准值	
		// 		}
		// 	}
			// status = 0;			
			// BEEP_H;
			// temp = SysTick_count;
			// break;
	}			
    //BEEP_H();
    
    return status;
}
