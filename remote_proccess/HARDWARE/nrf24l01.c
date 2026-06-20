/*******************************************************************
 *@brief 
 *@brief 
 *@time ZIN电子产品与技术 2017.1.8
 *@editor小南&zin
 *飞控爱好QQ群551883670,邮箱759421287@qq.com
 *非授权使用人员，禁止使用。禁止传阅，违者一经发现，侵权处理。
 *专业的飞控才是最好的飞控
 ******************************************************************/
#include "nrf24l01.h"
#include "SPI.h"
#include <string.h>

#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1


/* 额外返回值：用于区分“发送等待超时” */
#define NRF24L01_TIMEOUT    2


/* 软件超时计数：用于避免发送时死等 IRQ */
#define NRF24L01_TX_TIMEOUT_COUNT    60000U

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//24L01操作线   
#define NRF24L01_CSN_HIGH()    (GPIOB->BSRR = GPIO_Pin_12) // PB12
#define NRF24L01_CSN_LOW()     (GPIOB->BRR = GPIO_Pin_12)    // PB12
#define NRF24L01_CE_HIGH()     (GPIOA->BSRR = GPIO_Pin_11)    // PB11
#define NRF24L01_CE_LOW()      (GPIOA->BRR = GPIO_Pin_11)    // PB11

#define NRF24L01_IRQ_READ()   (GPIOA->IDR&GPIO_Pin_8)//IRQ主机数据输入 PB0




/**************************************************************************************/
//配对密码//各套件主控与遥控的匹配码相同才可以实现通信
//只购买一套者默认地址5
//手上有两套及以上着，实现一对一通信匹配码在这里更改，遥控与主控上会有地址序号来区分不同套，
//地址5
const uint8_t TX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE5};	//本地地址
const uint8_t RX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE5};	//接收地址RX_ADDR_P0 == RX_ADDR
//const uint8_t TX_ADDRESS[]= {0x20,0xE2,0x12,0xE4,0xe4};	//本地地址
//const uint8_t RX_ADDRESS[]= {0x20,0xE2,0x12,0xE4,0xe4};	//接收地址RX_ADDR_P0 == RX_ADDR
////地址4
//const uint8_t TX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE4};	//本地地址
//const uint8_t RX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE4};	//接收地址RX_ADDR_P0 == RX_ADDR
////地址3
//const uint8_t TX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE3};	//本地地址
//const uint8_t RX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE3};	//接收地址RX_ADDR_P0 == RX_ADDR
////地址2
//const uint8_t TX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE2};	//本地地址
//const uint8_t RX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE2};	//接收地址RX_ADDR_P0 == RX_ADDR
////地址1
//const uint8_t TX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE1};	//本地地址
//const uint8_t RX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE1};	//接收地址RX_ADDR_P0 == RX_ADDR
/**************************************************************************************/


/* ============================================================
 *  内部辅助函数
 * ============================================================ */

 /* 发送一个“仅命令”字节，不跟数据 */
 static void NRF24L01_Cmd(u8 cmd)
 {
	NRF24L01_CSN_LOW();                    //使能SPI传输
	SPI_RW(cmd);                           //发送命令字节
	NRF24L01_CSN_HIGH();                   //禁止SPI传输
 }

 /* 清除 STATUS 中断标志：写 1 清除 */
 static void NRF24L01_ClearIRQFlags(u8 flags)
 {
	NRF24L01_Write_Reg(SPI_WRITE_REG + STATUS, flags);
 }

 // 清空 TX FIFO
 static void NRF24L01_FlushTx(void)
 {
	NRF24L01_Cmd(FLUSH_TX);
 }

 // 清空 RX FIFO
 static void NRF24L01_FlushRx(void)
 {
	NRF24L01_Cmd(FLUSH_RX);
 }



//初始化24L01的IO口
void NRF24L01_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);    //使能GPIO的时钟  CE
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;          //NRF24L01 模块片选信号
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);   //使能GPIO的时钟 CSN
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;      
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	NRF24L01_CE_HIGH();                                    //初始化时先拉高
	NRF24L01_CSN_HIGH();                                   //初始化时先拉高

    //配置NRF2401的IRQ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU  ;     //上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_8);

	SPI_Config();                                //初始化SPI
	NRF24L01_CE_LOW(); 	                                //使能24L01
	NRF24L01_CSN_HIGH();                                   //SPI片选取消
}
//上电检测NRF24L01是否在位
//写5个数据然后再读回来进行比较，
//相同时返回值:0，表示在位;否则返回1，表示不在位	
u8 NRF24L01_Check(void)
{
	u8 txbuf[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
	u8 rxbuf[5];
	u8 i;

	// 写入测试地址
	NRF24L01_Write_Buf(SPI_WRITE_REG + TX_ADDR, txbuf, 5);

	// 读回测试地址
	NRF24L01_Read_Buf(TX_ADDR, rxbuf, 5);
	
	// 比较读取的数据
	for(i = 0; i < 5; i++)
	{
		if(rxbuf[i] != txbuf[i])
		{
			return FAILED; // 数据不一致
		}
	}
	return SUCCESS; // 检测通过	                                //NRF24L01在位
}	 	 
//通过SPI写寄存器
u8 NRF24L01_Write_Reg(u8 regaddr,u8 data)
{
	u8 status;	
    NRF24L01_CSN_LOW();                    //使能SPI传输
  	status =SPI_RW(regaddr); //发送寄存器号 
  	SPI_RW(data);            //写入寄存器的值
  	NRF24L01_CSN_HIGH();                    //禁止SPI传输	   
  	return(status);       		         //返回状态值
}
//读取SPI寄存器值 ，regaddr:要读的寄存器
u8 NRF24L01_Read_Reg(u8 regaddr)
{
	u8 reg_val;	    
	NRF24L01_CSN_LOW();                //使能SPI传输		
  	SPI_RW(regaddr);     //发送寄存器号
  	reg_val=SPI_RW(0XFF);//读取寄存器内容
  	NRF24L01_CSN_HIGH();                //禁止SPI传输		    
  	return(reg_val);                 //返回状态值
}	


//在指定位置读出指定长度的数据
//*pBuf:数据指针
//返回值,此次读到的状态寄存器值 
u8 NRF24L01_Read_Buf(u8 regaddr,u8 *pBuf,u8 datalen)
{
	u8 status;
	u8 i;

	if(pBuf == NULL)
	{
		return FAILED; // 参数错误
	}

	/* nRF24L01 单次最大 32 字节 */
	if(datalen == 0 || datalen > 32)
	{
		return FAILED; // 参数错误
	}

	NRF24L01_CSN_LOW();                    //使能SPI传输
  	status = SPI_RW(regaddr);                //发送寄存器值(位置),并读取状态值
  	for(i=0; i<datalen; i++)
	{
		pBuf[i] = SPI_RW(0xFF); //读取数据
	}
	NRF24L01_CSN_HIGH();                    //禁止SPI传输
	return status;                        //返回读到的状态值
}



//在指定位置写指定长度的数据
//*pBuf:数据指针
//返回值,此次读到的状态寄存器值
u8 NRF24L01_Write_Buf(u8 regaddr, u8 *pBuf, u8 datalen)
{
	u8 status;
	u8 i;

	if(pBuf == NULL)
	{
		return FAILED; // 参数错误
	}
	/* nRF24L01 单次最大 32 字节 */
	if(datalen == 0 || datalen > 32)
	{
		return FAILED; // 参数错误
	}

	NRF24L01_CSN_LOW();                    //使能SPI传输
	status = SPI_RW(regaddr);                //发送寄存器值(位置),并读取状态值
  	for(i=0; i<datalen; i++)
	{
		SPI_RW(*pBuf++); //写入数据
	}	
	NRF24L01_CSN_HIGH();                    //关闭SPI传输
	return status;                        //返回读到的状态值
}				   


//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:发送完成状况
u8 NRF24L01_TxPacket(u8 *txbuf)
{
		u8 state;
    u16 timeout;

    if (txbuf == NULL) {
        return FAILED;
    }

    /* 发送前确保 CE 低电平 */
    NRF24L01_CE_LOW();

    /* 清中断，清 FIFO，避免上一次残留影响这次发送 */
    NRF24L01_ClearIRQFlags(MAX_TX | TX_OK | RX_OK);
    NRF24L01_FlushTx();

    /* 把 32 字节数据写入 TX FIFO */
    NRF24L01_Write_Buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH);


    /* 拉高 CE，启动发送 */
    NRF24L01_CE_HIGH();

    /* 软件超时等待 IRQ 变低
       IRQ 低电平表示 TX 完成或达到最大重发次数 */
    timeout = NRF24L01_TX_TIMEOUT_COUNT;
    while (NRF24L01_IRQ_READ() != 0U) {
        if (timeout-- == 0U) {
            /* 超时，避免死锁 */
            NRF24L01_CE_LOW();
            NRF24L01_FlushTx();
            NRF24L01_ClearIRQFlags(MAX_TX | TX_OK | RX_OK);
            return NRF24L01_TIMEOUT;
        }
    }

    /* 发送完成后先拉低 CE，回到空闲 */
    NRF24L01_CE_LOW();

    /* 读取状态寄存器 */
    state = NRF24L01_Read_Reg(STATUS);

    /* 清除本次产生的中断标志
       注意：STATUS 是“写 1 清除”，这里只清真正关心的位 */
    NRF24L01_ClearIRQFlags(state & (MAX_TX | TX_OK | RX_OK));

    /* 判断结果 */
    if (state & MAX_TX) {
        /* 达到最大发送次数，清空 TX FIFO，准备下次发送 */
        NRF24L01_FlushTx();
        return FAILED; // 达到最大重发次数
    }

    if (state & TX_OK) {
        return SUCCESS;
    }

    return FAILED;
	// u8 state;   
	// Clr_NRF24L01_CE;
	// NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
	// Set_NRF24L01_CE;                                     //启动发送	   
	// while(READ_NRF24L01_IRQ!=0);                         //等待发送完成
	// state=NRF24L01_Read_Reg(STATUS);                     //读取状态寄存器的值	   
	// NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,state);      //清除TX_DS或MAX_RT中断标志
	// if(state&MAX_TX)                                     //达到最大重发次数
	// {
	// 	NRF24L01_Write_Reg(FLUSH_TX,0xff);               //清除TX FIFO寄存器 
	// 	return MAX_TX; 
	// }
	// if(state&TX_OK)                                      //发送完成
	// {
	// 	return TX_OK;
	// }
	// return 0xff;                                         //其他原因发送失败
}

//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:0，接收完成；其他，错误代码
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 state;

    if (rxbuf == NULL) {
        return FAILED;
    }

    /* 读取状态寄存器 */
    state = NRF24L01_Read_Reg(STATUS);

    /* 如果有接收中断 */
    if (state & RX_OK) {
        /* 先把 32 字节数据取出来 */
        NRF24L01_Read_Buf(RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH);

        /* 清除 RX 中断标志 */
        NRF24L01_ClearIRQFlags(RX_OK);

        /* 清空 RX FIFO，避免下次读取到残留数据 */
        NRF24L01_FlushRx();

        return SUCCESS;
    }

    /* 如果出现最大重发标志，也一并清掉，避免干扰后续状态判断 */
    if (state & MAX_TX) {
        NRF24L01_ClearIRQFlags(MAX_TX);
    }

    return FAILED;
	// u8 state;		    							      
	// state=NRF24L01_Read_Reg(STATUS);                //读取状态寄存器的值    	 
	// NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,state); //清除TX_DS或MAX_RT中断标志
	// if(state&RX_OK)                                 //接收到数据
	// {
	// 	NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
	// 	NRF24L01_Write_Reg(FLUSH_RX,0xff);          //清除RX FIFO寄存器 
	// 	return 0; 
	// }	   
	// return 1;                                      //没收到任何数据
}

//该函数初始化NRF24L01到RX模式
//设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
//当CE变高后,即进入RX模式,并可以接收数据了		   
//void RX_Mode(void)
//{
//	Clr_NRF24L01_CE;	  
//    //写RX节点地址
//  	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);

//    //使能通道0的自动应答    
//  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x01);    
//    //使能通道0的接收地址  	 
//  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01);
//    //设置RF通信频率		  
//  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,40);	     
//    //选择通道0的有效数据宽度 	    
//  	NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);
//    //设置TX发射参数,0db增益,1Mbps,低噪声增益开启   
//  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f);
//    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,PRIM_RX接收模式 
//  	NRF24L01_Write_Reg(SPI_WRITE_REG+NCONFIG, 0x0f); 
//    //CE为高,进入接收模式 
//  	Set_NRF24L01_CE;                                
//}			
//		


//该函数初始化NRF24L01到TX模式
//设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,
//选择RF频道,波特率和LNA HCURR PWR_UP,CRC使能
//当CE变高后,即进入RX模式,并可以接收数据了		   
//CE为高大于10us,则启动发送.	 
void TX_Mode(void)
{			
	    NRF24L01_CE_LOW();

    /* 清空 FIFO 和中断标志，避免旧状态干扰发送 */
    NRF24L01_FlushTx();
    NRF24L01_ClearIRQFlags(MAX_TX | TX_OK | RX_OK);

    /* 设置发送地址 */
    NRF24L01_Write_Buf(SPI_WRITE_REG + TX_ADDR, (u8 *)TX_ADDRESS, TX_ADR_WIDTH);

    /* 如果后续你打算开启自动应答，
       管道 0 的地址通常要与 TX_ADDR 相同。
       即使当前关闭自动应答，保留这句也不影响基本使用。 */
    NRF24L01_Write_Buf(SPI_WRITE_REG + RX_ADDR_P0, (u8 *)RX_ADDRESS, RX_ADR_WIDTH);

    /* 关闭自动应答，保持和你当前工程一致 */
    NRF24L01_Write_Reg(SPI_WRITE_REG + EN_AA, 0x00);

    /* 仅保留管道 0 配置，兼容性更好 */
    NRF24L01_Write_Reg(SPI_WRITE_REG + EN_RXADDR, 0x01);

    /* 地址宽度 */
    NRF24L01_Write_Reg(SPI_WRITE_REG + SETUP_AW, 0x03);

    /* 自动重发设置：关闭自动应答时基本无意义，但保留默认值 */
    NRF24L01_Write_Reg(SPI_WRITE_REG + SETUP_RETR, 0x1A);

    /* 选择 RF 通道，要与接收端一致 */
    NRF24L01_Write_Reg(SPI_WRITE_REG + RF_CH, 1);

    /* 射频参数 */
    NRF24L01_Write_Reg(SPI_WRITE_REG + RF_SETUP, 0x07);

    /* CONFIG：
       bit0 PRIM_RX = 0  -> 发送模式
       bit1 PWR_UP  = 1  -> 上电
       bit2 EN_CRC  = 1  -> 使能 CRC
       bit3 CRCO    = 1  -> 16 位 CRC
    */
    NRF24L01_Write_Reg(SPI_WRITE_REG + CONFIG, 0x0E);

    /* 发送模式下 CE 默认保持低，
       真正发送数据时在 TxPacket() 中拉高。 */
    NRF24L01_CE_LOW();											 
	// Clr_NRF24L01_CE;	    
    // //写TX节点地址 
  	// NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);    
    // //设置TX节点地址,主要为了使能ACK	  
  	// NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH); 

    // //使能通道0的自动应答    
  	// NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x00);     
    // //使能通道0的接收地址  
  	// NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01); 
    // //设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
  	// NRF24L01_Write_Reg(SPI_WRITE_REG+SETUP_RETR,0x1a);
    // //设置RF通道为40
  	// NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,1);       
    // //设置TX发射参数,0db增益,1Mbps,低噪声增益开启   
  	// NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x07);  //0x27  250K   0x07 1M
    // //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,PRIM_RX发送模式,开启所有中断
  	// NRF24L01_Write_Reg(SPI_WRITE_REG+NCONFIG,0x0e);    
    // // CE为高,10us后启动发送
	// Set_NRF24L01_CE;                                  
}	

void NRF24L01_init(void)
{
    uint8_t retry;

    NRF24L01_Configuration();

    /* 给模块一个稳定的初始化机会 */
    retry = 10;
    while (retry--) {
        if (NRF24L01_Check() == SUCCESS) {
            TX_Mode();
            return;
        }
    }

    /* 初始化失败：保持安全状态 */
    NRF24L01_CE_LOW();
    NRF24L01_CSN_HIGH();
	
}



/*********************END OF FILE******************************************************/
















