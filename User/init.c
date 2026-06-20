
#include "INIT.h"
#include "debug.h"
//#include "ALL_DEFINE.h"
//#include "pid.h"

//volatile uint32_t SysTick_count; //ϵͳʱ�����
//_st_Mpu MPU6050;   //MPU6050ԭʼ����
//_st_AngE Angle;    //��ǰ�Ƕ���ֵ̬
// _st_Remote Remote; //ң��ͨ��ֵ


//volatile ALL_flag_t ALL_flag; //ϵͳ��־λ������������־λ��



PidObject pidRateX; //�ڻ�PID����
PidObject pidRateY;
PidObject pidRateZ;

PidObject pidPitch; //�⻷PID����
PidObject pidRoll;
PidObject pidYaw;

void pid_param_Init(void); //PID���Ʋ�����ʼ������дPID�����ᱣ�����ݣ��������ɺ�ֱ���ڳ�������� ����¼���ɿ�

// void ALL_flag_set(uint8_t flag)
// {
// 	ALL_flag.unlock = flag & 0x01; //���ý�����־λ��flagΪ1��ʾ������flagΪ0��ʾ����
// }

// uint8_t ALL_flag_get(void)
// {
// 	return ALL_flag.unlock; //��ȡ������־λ������1��ʾ����������0��ʾ����
// }



// uint32_t get_SysTick(void)
// {
// 	return SysTick_count;
// }


/**************************************************************
 *  ����ϵͳ����ʹ�������ʼ��
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void ALL_Init(void)
{
	//USB_HID_Init();		//USB HID初始化，包含USB上位机通信和HID数据发送功能
	pid_param_Init();       //PID参数初始化

	 
	

	Delay_ms(100);

	LED_Init();              //LED闪灯初始化

	USART1_Config(9600);
	
	IIC_Init();             //I2C初始化	
//----------------------------------------	
// 水平静止标定，该功能只需要进行一次，不要每次进行。店家发货前已经进行一次标定了，标定完后会自动保存到MCU的FLASH中。
// 如需校准，重新打开即可，延时5S是为了插上电池后有充足的时间将飞行器放在地上进行水平静止标定。
//----------------------------------------		
//	USART1_Config();  //备用串口       
	
	Mpu_Init();              //MPU6050初始化


	if(NRF24L01_Init() != NRF24L01_OK)
	{
		LOG_E("NRF24L01 initialization failed\r\n");
	}
	else
	{
		LOG_I("NRF24L01 initialization succeeded\r\n");
	}

	//TIM2_PWM_Config();			//4路PWM初始化
	//PWM_Init();						//4路PWM初始化
	motor_init();						//电机初始化，包含电机PWM和GPIO初始化

	TIM3_Config();					//系统工作周期初始化 
	
}


/**************************************************************
 *  初始化PID参数
 * @brief 如果需要自己修改PID值，调这里就可以了
 * @param[out] 
 * @return     
 ***************************************************************/
void pid_param_Init(void)
{

//	pidRateX.kp = 1.5f;//huanbaoxian
//	pidRateY.kp = 1.5f;
//	pidRateZ.kp = 6.0f;
//	
//	pidRateX.ki = 0.04f;
//	pidRateY.ki = 0.04f;
//	pidRateZ.ki = 0.05f;	
//	
//	pidRateX.kd = 0.06f;
//	pidRateY.kd = 0.06f;
//	pidRateZ.kd = 0.4f;	
//	
//	pidPitch.kp = 4.0f;
//	pidRoll.kp = 4.0f;
//	pidYaw.kp = 3.0f;
	
//	pidRateX.kp = 2.0f;//1
//	pidRateY.kp = 2.0f;
//	pidRateZ.kp = 6.0f;
//	
//	pidRateX.ki = 0.05f;
//	pidRateY.ki = 0.05f;
//	pidRateZ.ki = 0.05f;	
//	
//	pidRateX.kd = 0.08f;
//	pidRateY.kd = 0.08f;
//	pidRateZ.kd = 0.5f;	
//	
//	pidPitch.kp = 7.0f;
//	pidRoll.kp = 7.0f;
//	pidYaw.kp = 4.0f;
	
	pidRateX.kp = 2.0f;//1
	pidRateY.kp = 2.0f;
	pidRateZ.kp = 6.0f;
	
	pidRateX.ki = 0.01f;
	pidRateY.ki = 0.01f;
	pidRateZ.ki = 0.05f;	
	
	pidRateX.kd = 0.08f;
	pidRateY.kd = 0.08f;
	pidRateZ.kd = 0.5f;	
	
	pidPitch.kp = 7.0f;
	pidRoll.kp = 7.0f;
	pidYaw.kp = 4.0f;		
}















