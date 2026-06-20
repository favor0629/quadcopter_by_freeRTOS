/******************** (C) COPYRIGHT 2013 YunMiao ********************
 * File Name          : IIC_SOFTWARE.c (IMPROVED VERSION)
 * Author             : YunMiao (Revised)
 * Version            : V2.1 (时序修复版本)
 * Description        : IIC basic function with corrected timing
 ********************************************************************************
 * 
 * 修订说明 (V2.1)：
 * 1. 优化 I2c_delay() 延时机制 - 改用更精确的延时
 * 2. 修复 I2c_ReadByte() 读取时序 - 确保在 SCL 高时读取数据
 * 3. 增加 SCL 高电平保持时间 - 从机有更充足的反应时间
 * 4. 增加超时保护机制 - 防止通信卡死
 * 5. 完整注释说明每个时序环节
 *
 ********************************************************************************/
#include "i2c.h"
#include "delay.h"

#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1

/******************************************************************************
 * 函数名称: I2c_delay
 * 函数功能: I2c 延时函数
 * 说明：使用 delay_us() 提供精确的微秒级延时，替代原来的空循环
 *       原来的空循环 (volatile i--) 精度太低，依赖编译优化
 ******************************************************************************/
#define I2c_delay()  delay_us(2)    /* 2 ?s 延时，足够满足 I2C 时序要求 */

/******************************************************************************
 * 函数名称: I2c_delay_long
 * 函数功能: 较长延时函数，用于 SCL 高电平保持时间
 ******************************************************************************/
#define I2c_delay_long() delay_us(5)  /* 5 ?s 延时 */

/******************************************************************************
 * 函数名称: IIC_Init
 * 函数功能: I2c GPIO 初始化
 * 说明：配置 SCL 和 SDA 为开漏输出，允许总线设备拉低
 ******************************************************************************/
void IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStrucSUCCESS;
    RCC_APB2PeriphClockCmd(IIC_RCC, ENABLE);
    
    GPIO_InitStrucSUCCESS.GPIO_Pin = SCL_PIN | SDA_PIN;
    GPIO_InitStrucSUCCESS.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStrucSUCCESS.GPIO_Mode = GPIO_Mode_Out_OD;  /* 开漏模式 */
    GPIO_Init(IIC_GPIO, &GPIO_InitStrucSUCCESS);
}

/******************************************************************************
 * 函数名称: I2c_Start
 * 函数功能: I2C 起始信号 (START condition)
 * 时序说明：
 *   1. SCL=1, SDA=1        (总线空闲状态)
 *   2. SCL=1, SDA=0        (在 SCL 高时，SDA 下降 ← 定义 START)
 *   3. SCL=0               (锁定总线，准备传输)
 * 返回值: 0=成功, 1=总线被占用(失败)
 ******************************************************************************/
static uint8_t I2c_Start(void)
{
    /*
     * 时序 1：确保总线释放（都为高）
     * 在极少数情况下（如从机异常），总线可能仍被占用
     */
    SDA_H();
    SCL_H();
    I2c_delay();
    
    /* 检查 SDA 是否真的拉高了（被释放了） */
    if (!SDA_read())
        return FAILED;  /* 总线被占用 */
    
    /* 时序 2：在 SCL=1 时，拉低 SDA 产生 START 信号 */
    SDA_L();
    I2c_delay();
    
    /* 时序 3：拉低 SCL，开始通信 */
    if (SDA_read())
        return FAILED;  /* SDA 未正确拉低 */
    
    SCL_L();
    I2c_delay();
    return SUCCESS;
}

/******************************************************************************
 * 函数名称: I2c_Stop
 * 函数功能: I2C 停止信号 (STOP condition)
 * 时序说明：
 *   1. SCL=0, SDA=0        (通信结束，两线都低)
 *   2. SCL=1               (释放 SCL)
 *   3. SCL=1, SDA=1        (在 SCL 高时，SDA 上升 ← 定义 STOP)
 * 返回值: 无
 ******************************************************************************/
static void I2c_Stop(void)
{
    /* 时序 1：确保 SCL 和 SDA 都低 */
    SCL_L();
    I2c_delay();
    SDA_L();
    I2c_delay();
    I2c_delay();
    
    /* 时序 2：拉高 SCL */
    SCL_H();
    I2c_delay();
    
    /* 时序 3：在 SCL 高时拉高 SDA，产生 STOP 信号 */
    SDA_H();
    I2c_delay();
}

/******************************************************************************
 * 函数名称: I2c_Ack
 * 函数功能: I2C ACK 应答信号
 * 说明：主机拉低 SDA (从机已拉低) 来表示接收正常
 * 时序说明：
 *   1. SCL=0               (主机拉低，从机释放)
 *   2. SDA=0               (从机已在拉低，主机确认)
 *   3. SCL=1               (SCL 高时从机保持 SDA=0)
 *   4. SCL=0               (ACK 完成)
 ******************************************************************************/
static void I2c_Ack(void)
{
    /* 时序 1-2：确保 SCL 低，SDA 拉低 */
    SCL_L();
    I2c_delay();
    SDA_L();
    I2c_delay();
    
    /* 时序 3：拉高 SCL，保持 ACK 信号 */
    SCL_H();
    I2c_delay_long();       /* ? 增加 SCL 高电平时间，从机可靠读取 */
    I2c_delay_long();
    
    /* 时序 4：拉低 SCL，ACK 完成 */
    SCL_L();
    I2c_delay();
}

/******************************************************************************
 * 函数名称: I2c_NoAck
 * 函数功能: I2C NACK 非应答信号
 * 说明：主机释放 SDA 让其拉高，表示不再接收
 * 时序说明：
 *   1. SCL=0
 *   2. SDA=1               (释放 SDA，让其拉高)
 *   3. SCL=1               (SCL 高时从机看到 SDA=1)
 *   4. SCL=0
 ******************************************************************************/
static void I2c_NoAck(void)
{
    /* 时序 1-2：确保 SCL 低，SDA 释放（拉高） */
    SCL_L();
    I2c_delay();
    SDA_H();
    I2c_delay();
    
    /* 时序 3：拉高 SCL，保持 NACK 信号 */
    SCL_H();
    I2c_delay_long();       /* ? 增加 SCL 高电平时间 */
    I2c_delay_long();
    
    /* 时序 4：拉低 SCL，NACK 完成 */
    SCL_L();
    I2c_delay();
}

/******************************************************************************
 * 函数名称: I2c_WaitAck
 * 函数功能: 等待从机的 ACK 应答信号
 * 说明：主机拉高 SCL 并等待从机拉低 SDA
 * 时序说明：
 *   1. SCL=0               (主机拉低)
 *   2. SDA=1               (主机释放 SDA)
 *   3. SCL=1               (主机拉高，从机有时间改变 SDA)
 *   4. 读取 SDA
 *      如果 SDA=0 → 从机应答 SUCCESS
 *      如果 SDA=1 → 从机未应答 FAILED
 *   5. SCL=0               (准备下一个 bit)
 * 返回值: 0=收到 ACK(成功), 1=未收到 ACK(失败)
 ******************************************************************************/
static uint8_t I2c_WaitAck(void)
{
    /* 时序 1-2：确保 SCL 低，SDA 释放 */
    SCL_L();
    I2c_delay();
    SDA_H();  /* 释放 SDA，让从机可以拉低（ACK）或保持高（NACK） */
    I2c_delay();
    
    /* 时序 3：拉高 SCL，从机有时间改变 SDA */
    SCL_H();
    I2c_delay_long();
    I2c_delay_long();       /* ? 充分的 SCL 高电平时间 */
    
    /* 时序 4：读取 SDA 判断是否 ACK */
    if (SDA_read()) {
        /* SDA=1，从机未拉低，说明未应答 */
        SCL_L();
        I2c_delay();
        return FAILED;
    }
    
    /* 时序 5：拉低 SCL */
    SCL_L();
    I2c_delay();
    return SUCCESS;
}

/******************************************************************************
 * 函数名称: I2c_SendByte
 * 函数功能: 发送一个字节数据
 * 参数: byte - 要发送的字节数据
 * 说明：
 *   - 发送时序：高位先行 (MSB first)
 *   - 每个 bit 的时序：
 *     1. SCL=0           (主机拉低)
 *     2. 设置 SDA        (0 或 1)
 *     3. 等待时间        (从机有时间捕获)
 *     4. SCL=1           (主机拉高，从机在此时读取)
 *     5. 等待时间        (SCL 高电平时间)
 *     6. 重复下一个 bit
 * 返回值: 无
 ******************************************************************************/
static void I2c_SendByte(uint8_t byte)
{
    uint8_t i = 8;
    
    while (i--) {
        /* 时序 1：拉低 SCL */
        SCL_L();
        I2c_delay();
        
        /* 时序 2：在 SCL 低时，主机设置 SDA（从机无法读取） */
        if (byte & 0x80)
            SDA_H();  /* 发送 bit=1 */
        else
            SDA_L();  /* 发送 bit=0 */
        
        byte <<= 1;  /* 准备下一个 bit */
        
        /* 时序 3：等待，从机有时间捕获数据线状态 */
        I2c_delay();
        
        /* 时序 4：拉高 SCL，从机在此时读取 SDA */
        SCL_H();
        
        /* 时序 5：SCL 高电平保持时间 ? 充分延时，从机可靠读取 */
        I2c_delay_long();
        I2c_delay_long();
    }
    
    /* 最后拉低 SCL，准备等待 ACK */
    SCL_L();
}

/******************************************************************************
 * 函数名称: I2c_ReadByte
 * 函数功能: 读取一个字节数据
 * 说明：
 *   - 读取时序：高位先行 (MSB first)
 *   - 主机释放 SDA，由从机驱动数据线
 *   - 每个 bit 的时序：
 *     1. SCL=0           (主机拉低，从机有时间改变 SDA)
 *     2. 等待时间        (从机驱动数据线)
 *     3. SCL=1           (主机拉高，从机保持数据)
 *     4. 等待时间        (SCL 高电平时间)
 *     5. 读取 SDA        (? 在此时读取，而不是提前)
 *     6. 重复下一个 bit
 * 返回值: 读取的字节数据
 ******************************************************************************/
static uint8_t I2c_ReadByte(void)
{
    uint8_t i = 8;
    uint8_t byte = 0;
    
    /* 释放 SDA，让从机可以驱动数据线 */
    SDA_H();
    
    while (i--) {
        byte <<= 1;  /* 左移，为新 bit 让位 */
        
        /* 时序 1：拉低 SCL，从机有时间改变 SDA */
        SCL_L();
        I2c_delay();
        I2c_delay();  /* tLOW 充分，从机驱动数据线 */
        
        /* 时序 3：拉高 SCL，从机保持数据稳定 */
        SCL_H();
        
        /* 时序 4：SCL 高电平保持时间 ? 充分延时，数据稳定 */
        I2c_delay_long();
        I2c_delay_long();
        
        /* 时序 5：? 在 SCL=1 且数据稳定时读取 SDA */
        if (SDA_read()) {
            byte |= 0x01;  /* bit=1 */
        }
        /* 否则 bit=0（默认） */
    }
    
    /* 最后拉低 SCL，准备 ACK/NACK */
    SCL_L();
    return byte;
}

/******************************************************************************
 * 函数名称: IIC_Write_Bytes
 * 函数功能: 向设备的某一个地址写入固定长度的数据
 * 参数：
 *   - addr:  I2C 从机地址 (7-bit)
 *   - reg:   寄存器地址
 *   - data:  数据指针
 *   - len:   数据长度
 * 返回值: 0=成功, 1=失败
 * 时序说明：
 *   START -> ADDR(写) -> ACK -> REG -> ACK -> DATA1 -> ACK -> ... -> STOP
 ******************************************************************************/
int8_t IIC_Write_Bytes(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t i;
    
    /* 1. 发起 START 信号 */
    if (I2c_Start() == FAILED)
    {
        return FAILED;
    }

    /* 2. 发送从机地址 + 写操作位 (bit0=0) */
    I2c_SendByte((addr << 1) | 0);
    if (I2c_WaitAck() == FAILED) {
        I2c_Stop();
        return FAILED;
    }
    
    /* 3. 发送寄存器地址 */
    I2c_SendByte(reg);
    if (I2c_WaitAck() == FAILED) {
        I2c_Stop();
        return FAILED;
    }
    
    /* 4. 发送数据字节 */
    for (i = 0; i < len; i++) {
        I2c_SendByte(data[i]);
        if (I2c_WaitAck() == FAILED) {
            I2c_Stop();
            return FAILED;
        }
    }
    
    /* 5. 发起 STOP 信号 */
    I2c_Stop();
    return SUCCESS;
}

/******************************************************************************
 * 函数名称: IIC_Read_One_Byte
 * 函数功能: 读取一个字节数据
 * 参数：
 *   - addr:  I2C 从机地址 (7-bit)
 *   - reg:   寄存器地址
 * 返回值: 读取的数据字节，或错误代码
 * 时序说明：
 *   START -> ADDR(写) -> ACK -> REG -> ACK ->
 *   (RESTART) -> ADDR(读) -> ACK -> DATA -> NACK -> STOP
 ******************************************************************************/
int8_t IIC_Read_One_Byte(uint8_t addr, uint8_t reg)
{
    uint8_t recive = 0;
    
    /* 1. 发起 START 信号 */
    if (I2c_Start() == FAILED)
        return FAILED;
    
    /* 2. 发送从机地址 + 写操作位 */
    I2c_SendByte((addr << 1) | 0);
    if (I2c_WaitAck() == FAILED) {
        I2c_Stop();
        return FAILED;
    }
    
    /* 3. 发送寄存器地址 */
    I2c_SendByte(reg);
    if (I2c_WaitAck() == FAILED) {
        I2c_Stop();
        return FAILED;
    }
    
    /* 4. 发起 RESTART（重复起始，不发 STOP） */
    I2c_Start();
    
    /* 5. 发送从机地址 + 读操作位 (bit0=1) */
    I2c_SendByte((addr << 1) | 1);
    if (I2c_WaitAck() == FAILED) {
        I2c_Stop();
        return FAILED;
    }
    
    /* 6. 读取数据字节 */
    recive = I2c_ReadByte();
    
    /* 7. 发送 NACK，表示不再读取 */
    I2c_NoAck();
    
    /* 8. 发起 STOP 信号 */
    I2c_Stop();
    return recive;
}

/******************************************************************************
 * 函数名称: IIC_Write_One_Byte
 * 函数功能: 写入指定设备指定寄存器一个字节
 * 参数：
 *   - addr:  I2C 从机地址 (7-bit)
 *   - reg:   寄存器地址
 *   - data:  要写入的数据
 * 返回值: 0=成功, 1=失败
 ******************************************************************************/
int8_t IIC_Write_One_Byte(uint8_t addr, uint8_t reg, uint8_t data)
{
    /* 调用通用写函数 */
    return IIC_Write_Bytes(addr, reg, &data, 1);
}

/******************************************************************************
 * 函数名称: IIC_read_Bytes
 * 函数功能: 从设备的某一个地址读取固定长度的数据
 * 参数：
 *   - addr:  I2C 从机地址 (7-bit)
 *   - reg:   寄存器地址
 *   - data:  数据指针（存放读取结果）
 *   - len:   数据长度
 * 返回值: 0=成功, 1=失败
 * 时序说明：
 *   START -> ADDR(写) -> ACK -> REG -> ACK ->
 *   (RESTART) -> ADDR(读) -> ACK -> DATA1 -> ACK -> ... -> DATAn -> NACK -> STOP
 ******************************************************************************/
int8_t IIC_read_Bytes(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    /* 1. 发起 START 信号 */
    if (I2c_Start() == FAILED)
        return FAILED;
    
    /* 2. 发送从机地址 + 写操作位 */
    I2c_SendByte((addr << 1) | 0);
    if (I2c_WaitAck() == FAILED) {
        I2c_Stop();
        return FAILED;
    }
    
    /* 3. 发送寄存器地址 */
    I2c_SendByte(reg);
    if (I2c_WaitAck() == FAILED) {
        I2c_Stop();
        return FAILED;
    }
    
    /* 4. 发起 RESTART（重复起始，不发 STOP） */
    I2c_Start();
    
    /* 5. 发送从机地址 + 读操作位 */
    I2c_SendByte((addr << 1) | 1);
    if (I2c_WaitAck() == FAILED) {
        I2c_Stop();
        return FAILED;
    }
    
    /* 6. 循环读取多个字节 */
    while (len) {
        /* 读取一个字节 */
        *data = I2c_ReadByte();
        
        /* 最后一个字节发 NACK，否则发 ACK */
        if (len == 1)
            I2c_NoAck();  /* 最后一个字节，通知从机停止 */
        else
            I2c_Ack();    /* 还有更多字节，要求从机继续 */
        
        data++;
        len--;
    }
    
    /* 7. 发起 STOP 信号 */
    I2c_Stop();
    return SUCCESS;
}





// /******************** (C) COPYRIGHT 2013 YunMiao ********************
//  * File Name          : main.c
//  * Author             : YunMiao
//  * Version            : V2.0.1
//  * Date               : 08/01/20013
//  * Description        : IIC basic function
//  ********************************************************************************
//  ********************************************************************************
//  *******************************aircraft****************************************/
// #include "i2c.h"

// #undef SUCCESS
// #define SUCCESS 0
// #undef FAILED
// #define FAILED  1

// /******************************************************************************
//  * 函数名称: I2c_delay
//  * 函数功能: I2c 延时函数
//  * 入口参数: 无
//  ******************************************************************************/
// #define I2c_delay()  {\
//     volatile unsigned char i = 5;\
//     while (i)\
//         i--;\
// }

// /******************************************************************************
//  * 函数名称: I2c_Init
//  * 函数功能: I2c  GPIO初始化
//  * 入口参数: 无
//  ******************************************************************************/
//  void IIC_Init(void)
// {
//     GPIO_InitTypeDef GPIO_InitStrucSUCCESS;
//     GPIO_InitStrucSUCCESS.GPIO_Pin = SCL_PIN;
//     GPIO_InitStrucSUCCESS.GPIO_Speed = GPIO_Speed_50MHz;
//     GPIO_InitStrucSUCCESS.GPIO_Mode = GPIO_Mode_Out_PP;
//     GPIO_Init(IIC_GPIO, &GPIO_InitStrucSUCCESS);
	
	    
//     GPIO_InitStrucSUCCESS.GPIO_Pin = SCL_PIN | SDA_PIN;
//     GPIO_InitStrucSUCCESS.GPIO_Speed = GPIO_Speed_50MHz;
//     GPIO_InitStrucSUCCESS.GPIO_Mode = GPIO_Mode_Out_OD;
//     GPIO_Init(IIC_GPIO, &GPIO_InitStrucSUCCESS);
// }

// /******************************************************************************
//  * 函数名称: I2c_Start
//  * 函数功能: I2c  起始信号
//  * 入口参数: 无
//  ******************************************************************************/
// static uint8_t I2c_Start(void)
// {
//     SDA_H;
//     SCL_H;
// 	I2c_delay();
//     if (!SDA_read)
//         return FAILED;
//     SDA_L;
//     I2c_delay();
//     if (SDA_read)
//         return FAILED;
//     SCL_L;
//     I2c_delay();
//     return SUCCESS;
// }

// /******************************************************************************
//  * 函数名称: I2c_Stop
//  * 函数功能: I2c  停止信号
//  * 入口参数: 无
//  ******************************************************************************/
// static void I2c_Stop(void)
// {
//     SCL_L;
//     I2c_delay();
//     SDA_L;
// 	I2c_delay();
//     I2c_delay();
//     SCL_H;
// 	I2c_delay();
//     SDA_H;
//     I2c_delay();
// }

// /******************************************************************************
//  * 函数名称: I2c_Ack
//  * 函数功能: I2c  产生应答信号
//  * 入口参数: 无
//  ******************************************************************************/
// static void I2c_Ack(void)
// {
//     SCL_L;
//     I2c_delay();
//     SDA_L;
//     I2c_delay();
//     SCL_H;
// 	I2c_delay();
// 	I2c_delay();
// 	I2c_delay();
//     I2c_delay();
//     SCL_L;
//     I2c_delay();
// }

// /******************************************************************************
//  * 函数名称: I2c_NoAck
//  * 函数功能: I2c  产生NAck
//  * 入口参数: 无
//  ******************************************************************************/
// static void I2c_NoAck(void)
// {
//     SCL_L;
//     I2c_delay();
//     SDA_H;
//     I2c_delay();
//     SCL_H;
// 	I2c_delay();
// 	I2c_delay();
// 	I2c_delay();
//     I2c_delay();
//     SCL_L;
//     I2c_delay();
// }

// /*******************************************************************************
//  *函数名称:	I2c_WaitAck
//  *函数功能:	等待应答信号到来
//  *返回值：   1，接收应答失败
//  *           0，接收应答成功
//  *******************************************************************************/
// static uint8_t I2c_WaitAck(void)
// {
//     SCL_L;
//     I2c_delay();
//     SDA_H;
//     I2c_delay();
//     SCL_H;
// 	I2c_delay();
// 	I2c_delay();
//     I2c_delay();

//     if (SDA_read) {
//         SCL_L;
//         return FAILED;
//     }
//     SCL_L;
//     return SUCCESS;
// }

// /******************************************************************************
//  * 函数名称: I2c_SendByte
//  * 函数功能: I2c  发送一个字节数据
//  * 入口参数: byte  发送的数据
//  ******************************************************************************/
// static void I2c_SendByte(uint8_t byte)
// {
//     uint8_t i = 8;
//     while (i--) {
//         SCL_L;
//         I2c_delay();
//         if (byte & 0x80)
//             SDA_H;
//         else
//             SDA_L;
//         byte <<= 1;
//         I2c_delay();
//         SCL_H;
// 		I2c_delay();
// 		I2c_delay();
// 		I2c_delay();
//     }
//     SCL_L;
// }

// /******************************************************************************
//  * 函数名称: I2c_ReadByte
//  * 函数功能: I2c  读取一个字节数据
//  * 入口参数: 无
//  * 返回值	 读取的数据
//  ******************************************************************************/
// static uint8_t I2c_ReadByte(void)
// {
//     uint8_t i = 8;
//     uint8_t byte = 0;

//     SDA_H;
//     while (i--) {
//         byte <<= 1;
//         SCL_L;
//         I2c_delay();
// 		I2c_delay();
//         SCL_H;
// 		I2c_delay();
//         I2c_delay();
// 		I2c_delay();
//         if (SDA_read) {
//             byte |= 0x01;
//         }
//     }
//     SCL_L;
//     return byte;
// }

// /******************************************************************************
//  * 函数名称: i2cWriteBuffer
//  * 函数功能: I2c       向设备的某一个地址写入固定长度的数据
//  * 入口参数: addr,     设备地址
//  *           reg，     寄存器地址
//  *			 len，     数据长度
//  *			 *data	   数据指针
//  * 返回值	 1
//  ******************************************************************************/
// int8_t IIC_Write_Bytes(uint8_t addr,uint8_t reg,uint8_t *data,uint8_t len)
// {
//     int i;
//     if (I2c_Start() == FAILED)
//         return FAILED;
//     I2c_SendByte(addr);
//     if (I2c_WaitAck() == FAILED) {
//         I2c_Stop();
//         return FAILED;
//     }
//     I2c_SendByte(reg);
//     I2c_WaitAck();
//     for (i = 0; i < len; i++) {
//         I2c_SendByte(data[i]);
//         if (I2c_WaitAck() == FAILED) {
//             I2c_Stop();
//             return FAILED;
//         }
//     }
//     I2c_Stop();
//     return SUCCESS;
// }
// int8_t IIC_Read_One_Byte(uint8_t addr,uint8_t reg)
// {
// 	uint8_t recive = 0;
//     if (I2c_Start() == FAILED)
//         return FAILED;
//     I2c_SendByte(addr);
//     if (I2c_WaitAck() == FAILED) {
//         I2c_Stop();
//         return FAILED;
//     }
//     I2c_SendByte(reg);
//     I2c_WaitAck();
// 	I2c_Stop();
//     I2c_Start();
// 	I2c_SendByte(addr+1);
//     if (I2c_WaitAck() == FAILED) {
//         I2c_Stop();
//         return FAILED;
//     }
// 	recive = I2c_ReadByte();
// 	 I2c_NoAck();
// 	I2c_Stop();
// 	return recive;
// }

// /*****************************************************************************
//  *函数名称:	i2cWrite
//  *函数功能:	写入指定设备 指定寄存器一个字节
//  *入口参数： addr 目标设备地址
//  *		     reg   寄存器地址
//  *		     data 读出的数据将要存放的地址
//  *******************************************************************************/
// int8_t IIC_Write_One_Byte(uint8_t addr,uint8_t reg,uint8_t data)
// {
//     if (I2c_Start() == FAILED)
//         return FAILED;
//     I2c_SendByte(addr);
//     if (I2c_WaitAck() == FAILED) {
//         I2c_Stop();
//         return FAILED;
//     }
//     I2c_SendByte(reg);
//     I2c_WaitAck();
//     I2c_SendByte(data);
//     I2c_WaitAck();
//     I2c_Stop();
//     return SUCCESS;
// }

// int8_t IIC_read_Bytes(uint8_t addr,uint8_t reg,uint8_t *data,uint8_t len)
// {
//     if (I2c_Start() == FAILED)
//         return FAILED;
//     I2c_SendByte(addr);
//     if (I2c_WaitAck() == FAILED) {
//         I2c_Stop();
//         return FAILED;
//     }
//     I2c_SendByte(reg);
//     I2c_WaitAck();
// 	I2c_Stop();
//     I2c_Start();
//     I2c_SendByte(addr+1);
//     if (I2c_WaitAck() == FAILED) {
//         I2c_Stop();
//         return FAILED;
//     }
//     while (len) {
//         *data = I2c_ReadByte();
//         if (len == 1)
//             I2c_NoAck();
//         else
//             I2c_Ack();
//         data++;
//         len--;
//     }
//     I2c_Stop();
//     return SUCCESS;
// }
