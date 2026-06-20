/**************************************************************
 * 
 * @brief
 *   ZIN-7套件
 *   飞控爱好群551883670
 *   淘宝地址：https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 *
 * 说明：
 *   这份代码实现的是“软件模拟 SPI”（也叫位带式 SPI、GPIO 模拟 SPI）。
 *   它不是 STM32 硬件 SPI 外设，而是通过普通 GPIO 手动拉高/拉低时钟和数据线，
 *   实现 SPI 通信时序。
 ***************************************************************/

#include "spi.h"

/*
 * 下面这些宏用于直接控制 GPIOB 的引脚电平。
 *
 * 这里用的是 STM32F1 的寄存器方式：
 *   BSRR：置位寄存器，写 1 可将对应引脚拉高
 *   BRR ：复位寄存器，写 1 可将对应引脚拉低
 *
 * 注意：
 *   这几个宏都没有加 do...while(0)，
 *   在复杂语句里使用时要注意语法安全性。
 */

/* 将 PB13 拉高，作为 SPI 时钟 SCK */
#define  SCK_H()   GPIOB->BSRR = GPIO_Pin_13

/* 将 PB13 拉低，作为 SPI 时钟 SCK */
#define  SCK_L()   GPIOB->BRR  = GPIO_Pin_13

/* 将 PB15 拉高，作为 SPI 主机输出数据 MOSI */
#define  MOSI_H()  GPIOB->BSRR = GPIO_Pin_15

/* 将 PB15 拉低，作为 SPI 主机输出数据 MOSI */
#define  MOSI_L()  GPIOB->BRR  = GPIO_Pin_15

/*
 * 读取 PB14 的电平，作为 SPI 从机输出数据 MISO。
 *
 * 如果 PB14 为高电平，则返回 1，否则返回 0。
 */
#define  MISO   ((GPIOB->IDR & GPIO_Pin_14) ? 1 : 0)


/*=============================================================================
 * 函数名称：SPI_Config
 * 函数功能：初始化软件模拟 SPI 所需的 GPIO
 * 参数说明：无
 * 返回值：无
 *
 * 引脚分配：
 *   PB13 -> SCK  (输出)
 *   PB15 -> MOSI (输出)
 *   PB14 -> MISO (输入)
 *   PB12 -> CS   (片选，代码里只预置为高电平)
 *=============================================================================*/
void SPI_Config(void)  // IO 初始化配置
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 使能 GPIOB 时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /*
     * 片选信号 CS 预置为高电平
     *
     * 通常 SPI 器件在未通信时，CS 保持高电平。
     * 当开始通信时，CS 拉低；通信结束后，CS 拉高。
     *
     * 注意：
     * 这里直接对 PB12 置位，但代码中并没有看到 PB12 的输出模式配置。
     * 如果 PB12 真正作为 CS 使用，最好显式把 PB12 配置成推挽输出。
     */
    GPIO_SetBits(GPIOB, GPIO_Pin_12);  // NRF_CS 预置为高

    /* ------------------------------------------------------------
     * 配置 PB13 为推挽输出，用作 SPI 时钟 SCK
     * ------------------------------------------------------------
     * 推挽输出适合主动输出高低电平。
     * 模拟 SPI 的时钟需要由主机控制，所以这里配置为输出。
     */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;     // SCK
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* ------------------------------------------------------------
     * 配置 PB15 为推挽输出，用作 MOSI
     * ------------------------------------------------------------
     * MOSI 是主机输出到从机的数据线，所以必须配置为输出。
     */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;     // MOSI
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* ------------------------------------------------------------
     * 配置 PB14 为浮空输入，用作 MISO
     * ------------------------------------------------------------
     * MISO 是从机输出到主机的数据线，所以主机侧应配置为输入。
     * 这里使用浮空输入。
     *
     */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;     // MISO
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // MISO 使用浮空输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /*
     * 初始化时把时钟拉低，
     * MOSI 先置高，作为默认空闲状态。
     *
     * 为什么要这样做：
     *   - SCK 拉低：让 SPI 处于初始空闲状态
     *   - MOSI 置高：给出一个默认电平，避免悬空
     *
     * 具体初始电平是否合理，还要结合 SPI 模式来判断。
     */
    SCK_L();
    MOSI_H();
}


/*=============================================================================
 * 函数名称：SPI_RW
 * 函数功能：SPI 单字节发送并同时接收
 * 参数说明：
 *   byte：待发送的 8 位数据
 * 返回值：
 *   返回从 MISO 线上读到的 8 位数据
 *
 * 说明：
 *   SPI 是全双工通信：
 *     主机每发送 1 位数据的同时，也会从从机接收 1 位数据。
 *
 *   这里采用的是“按位移位”的方式：
 *     - 发送 byte 的最高位
 *     - 时钟翻转一次
 *     - 采样 MISO
 *     - 然后 byte 左移一位，准备下一位
 *
 *   最终返回 Temp，即从机返回的 8 位数据。
 *=============================================================================*/
uint8_t SPI_RW(uint8_t byte)
{
    uint8_t i;
    uint8_t Temp = 0x00;   // 用于保存接收到的 8 位数据

    for (i = 0; i < 8; i++)
    {
        /*
         * 先将 SCK 拉低。
         * 这里的时序写法说明该代码是在模拟 SPI 时钟边沿。
         * 具体是在上升沿采样还是下降沿采样，要由 SPI 模式（CPOL/CPHA）决定。
		 * 这里的时序实现属于软件模拟，必须和从设备时序匹配。
         */
        SCK_L(); // sclk = 0

        /*
         * 发送 byte 的最高位到 MOSI
         *
         * 如果 byte 的最高位为 1，则 MOSI 拉高；
         * 如果最高位为 0，则 MOSI 拉低。
         */
        if (byte & 0x80)
        {
            MOSI_H(); // 输出 1
        }
        else
        {
            MOSI_L(); // 输出 0
        }

        /*
         * 左移一位，下一轮要发送的位移到最高位
         *
         * 例如：
         *   byte = 1010 0000
         *   左移后变成 0100 0000
         */
        byte <<= 1;

        /*
         * 接收值左移一位，给新读到的位腾出最低位位置
         *
         * 例如：
         *   Temp = 0000 0001
         *   左移后 = 0000 0010
         *   然后若本次读到 MISO=1，再加 1，变成 0000 0011
         */
        Temp <<= 1;

        /*
         * SCK 拉高，形成时钟上升沿或高电平采样窗口
         *
         * 在 SPI 中，具体是在上升沿采样还是下降沿采样，
         * 要由 SPI 模式（CPOL/CPHA）决定。
         * 这里的时序实现属于软件模拟，必须和从设备时序匹配。
         */
        SCK_H();

        /*
         * 在时钟有效边沿后读取 MISO
         *
         * 如果 MISO 当前为高电平，则把 Temp 的最低位加 1。
         */
        if (MISO)
        {
            Temp++;
        }

        /*
         * 再次把 SCK 拉低，结束本位传输周期
         */
        SCK_L();
    }

    /*
     * 返回接收到的 8 位数据
     */
    return Temp;
}



// /*******************************************************************
//  *@brief 
//  *@brief 
//  *@time ZIN电子产品与技术 2017.1.8
//  *@editor小南&zin
//  *飞控爱好QQ群551883670,邮箱759421287@qq.com
//  *非授权使用人员，禁止使用。禁止传阅，违者一经发现，侵权处理。
//  *
//  ******************************************************************/
// #include "spi.h"

// #define  SCK_H  GPIOB->BSRR=GPIO_Pin_13  //SCK拉高
// #define  SCK_L  GPIOB->BRR=GPIO_Pin_13  //SCK拉高
// #define  MOSI_H  GPIOB->BSRR=GPIO_Pin_15  //MOSI拉高
// #define  MOSI_L  GPIOB->BRR=GPIO_Pin_15  //MOSI拉高
// #define  MISO  ((GPIOB->IDR&GPIO_Pin_14)?1:0)  //读取MISO

// //#define  SCK_H  GPIO_SetBits(GPIOB, GPIO_Pin_13)  //SCK拉高
// //#define  SCK_L  GPIO_ResetBits(GPIOB, GPIO_Pin_13)  //SCK拉高

// //#define  MOSI_H  GPIO_SetBits(GPIOB, GPIO_Pin_15)  //MOSI拉高
// //#define  MOSI_L  GPIO_ResetBits(GPIOB, GPIO_Pin_15)  //MOSI拉高

// //#define  MISO   GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14)  //读取MISO

// void SPI_Config(void)//io初始化配置
// {
// 	GPIO_InitTypeDef GPIO_InitStructure;
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
// 	GPIO_SetBits(GPIOB, GPIO_Pin_12); //NRF_CS预置为高 

// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;//SCK
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// 	GPIO_Init(GPIOB, &GPIO_InitStructure);

// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;//MOSI
// 	GPIO_Init(GPIOB, &GPIO_InitStructure);

// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 ;//MISO
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //MOSI要用模拟输入
// 	GPIO_Init(GPIOB,&GPIO_InitStructure);
// 	SCK_L;
// 	MOSI_H;
// }




// uint8_t SPI_RW(uint8_t byte)
// {
// 		uint8_t i;

// 		uint8_t Temp=0x00;

// 		for (i = 0; i < 8; i++)

// 		{

// 				SCK_L;//sclk = 0;//先将时钟拉高

// 				if (byte&0x80)
// 				{
// 						MOSI_H; // //SO=1
// 				}
// 				else
// 				{
// 						MOSI_L;// //SO=0
// 				}

// 				byte <<= 1;
// 				Temp<<=1;
				
// 				SCK_H; //拉低时钟
				
// 				if(MISO) //读到1时
// 				{
// 					Temp++;
// 				}
// 				SCK_L;	
// 		}
// 		return Temp; //返回读到miso输入的值

// }


