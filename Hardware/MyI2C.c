#include "MyI2C.h"
#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/******************宏定义***********************/
/* 使用 GPIOB 模拟 I2C 通信，PB10 为 SCL，PB11 为 SDA */
#define I2C_PORT        GPIOB
#define I2C_SCL_PIN     GPIO_Pin_10
#define I2C_SDA_PIN     GPIO_Pin_11
#define I2C_PORT_RCC    RCC_APB2Periph_GPIOB


/* ===================== 内部互斥保护 ===================== */
/*
 * 软 I2C 是共享资源，必须保证同一时刻只有一个任务访问总线。
 * 这里采用“总线级互斥 + 当前拥有者记录”的方式。
 * 这样可以支持同一任务内的 repeated start，不会因为再次 Start 而自锁。
 */
#if (configUSE_MUTEXES == 1)
static SemaphoreHandle_t s_i2c_mutex = NULL; // I2C 互斥锁
#endif

static TaskHandle_t s_i2c_onwer = NULL; // 当前拥有 I2C 总线访问权的任务句柄
static uint8_t s_i2c_bus_locked = 0; // 总线锁定标志，0表示未锁定，1表示锁定

/******************内部函数定义***********************/
// 延时函数
static void MyI2C_Delay(void)
{
    volatile uint8_t i;
    for(i = 0; i < 8; ++i)
    {
        __NOP();
    }
}

/******************底层引脚操作***********************/

/**
 * @brief I2C 写SCL电平
 * @param BitValue 0表示拉低SCL，1表示释放SCL（外部上拉）
 * @retval None
 */
static void MyI2C_W_SCL(uint8_t BitValue)
{
    GPIO_WriteBit(I2C_PORT, I2C_SCL_PIN, (BitAction)(BitValue));
}

/**
 * @brief I2C 写SDA电平
 * @param BitValue 0表示拉低SDA，1表示释放SDA（外部上拉）
 * @retval None
 */
static void MyI2C_W_SDA(uint8_t BitValue)
{
    GPIO_WriteBit(I2C_PORT, I2C_SDA_PIN, (BitAction)(BitValue));
}

/**
  * @brief  I2C 读 SDA 电平
  * @retval 0 或 1
  */
 static uint8_t MyI2C_R_SDA(void)
 {
    return (uint8_t)GPIO_ReadInputDataBit(I2C_PORT, I2C_SDA_PIN);
 }

/******************总线互斥控制*********************** */
/**
  * @brief  获取 I2C 总线
  * @note   只能在任务上下文调用，不能在中断里调用
  */
 static void MyI2C_LockBus(void)
 {
#if (configUSE_MUTEXES == 1)
    TaskHandle_t current_task = xTaskGetCurrentTaskHandle(); // 获取当前任务句柄

    /**
     * 如果当前任务已经持有总线，说明这是同一事务中的再次start
     * 这种情况不需要再次获取互斥锁，直接返回即可，否则会死锁
     */
    taskENTER_CRITICAL();
    if(s_i2c_bus_locked != 0U && s_i2c_onwer == current_task)
    {
        taskEXIT_CRITICAL();
        return; // 已经持有总线，直接返回
    }
    taskEXIT_CRITICAL();

    // 这是第一次获取总线，需要获取互斥锁
    if(s_i2c_mutex != NULL)
    {
        xSemaphoreTake(s_i2c_mutex, portMAX_DELAY); // 获取互斥锁，等待直到获取成功
    }

    // 访问临界区资源
    taskENTER_CRITICAL();
    s_i2c_onwer = current_task; // 记录当前拥有者
    s_i2c_bus_locked = 1U; // 锁定总线
    taskEXIT_CRITICAL();
#else
    // 不使用互斥锁，直接标记总线被占用
    (void)0;
#endif
 }


 /**
  * @brief 释放 I2C 总线
  * @note 只能在任务上下文调用，不能在中断里调用
  */
static void MyI2C_UnlockBus(void)
{
#if (configUSE_MUTEXES == 1)
    TaskHandle_t current_task = xTaskGetCurrentTaskHandle(); // 获取当前任务句柄

    taskENTER_CRITICAL();
    if(s_i2c_bus_locked != 0U && s_i2c_onwer == current_task)
    {
        s_i2c_onwer = NULL; // 清除拥有者记录
        s_i2c_bus_locked = 0U; // 解锁总线
        taskEXIT_CRITICAL();

        if(s_i2c_mutex != NULL)
        {
            xSemaphoreGive(s_i2c_mutex); // 释放互斥锁
        }
    }
    else
    {
        taskEXIT_CRITICAL();
        // 当前任务没有持有总线，非法调用，直接返回
    }
#else
    // 不使用互斥锁，直接标记总线空闲
    (void)0;
#endif
}


/******************外部函数定义***********************/

/**
 * @brief I2C 初始化函数
 * @retval None
 */
void MyI2C_Init(void)
{
    // 开启时钟
    RCC_APB2PeriphClockCmd(I2C_PORT_RCC, ENABLE);

    // 初始化GPIO，开漏输出，
    GPIO_InitTypeDef gpio_init_structure;
    gpio_init_structure.GPIO_Mode = GPIO_Mode_Out_OD; // 开漏输出
    gpio_init_structure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
    gpio_init_structure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(I2C_PORT, &gpio_init_structure);

    // 释放总线，进入空闲状态
    GPIO_SetBits(I2C_PORT, I2C_SCL_PIN | I2C_SDA_PIN); // 释放 SCL 和 SDA，进入空闲状态

    #if (configUSE_MUTEXES == 1)
    {
        s_i2c_mutex = xSemaphoreCreateMutex(); // 创建互斥锁
    }
    #endif

    s_i2c_onwer = NULL;
    s_i2c_bus_locked = 0;
}


/******************协议层**************************** */

/**
 * @brief I2C 起始
 */
void MyI2C_Start(void)
{
    // 获取总线访问权
    MyI2C_LockBus();

    // 产生起始条件：SDA 由高变低，SCL 保持高电平, 产生下降沿
    MyI2C_W_SDA(1);
    MyI2C_W_SCL(1);
    MyI2C_Delay();

    MyI2C_W_SDA(0);
    MyI2C_Delay();

    MyI2C_W_SCL(0);
    MyI2C_Delay();
}


/**
 * @brief I2C 停止
 */
void MyI2C_Stop(void)
{
    // 终止条件，SCL高时，SDA由低变高，产生上升沿
    MyI2C_W_SDA(0);
    MyI2C_Delay();

    MyI2C_W_SCL(1);
    MyI2C_Delay();

    MyI2C_W_SDA(1);
    MyI2C_Delay();

    // 释放总线访问权
    MyI2C_UnlockBus();
}


/***
 * @brief I2C 发送一个字节
 * @param Byte 要发送的字节
 */
void MyI2C_SendByte(uint8_t Byte)
{
    uint8_t i;

    for(i = 0; i < 8; ++i)
    {
        MyI2C_W_SDA((Byte & (0x80 >> i)) ? 1U : 0U); // 从最高位开始发送
        MyI2C_Delay();

        MyI2C_W_SCL(1); // 拉高 SCL，数据被接收
        MyI2C_Delay();

        MyI2C_W_SCL(0); // 拉低 SCL，准备发送下一位
        MyI2C_Delay();
    }
}


/**
 * @brief I2C 接收一个字节
 * @retval 接收到的字节
 */
uint8_t MyI2C_ReceiveByte(void)
{
    uint8_t i;
    uint8_t received_byte = 0x00;

    // 主机释放 SDA 线，准备接收数据
    MyI2C_W_SDA(1); // 释放 SDA，进入输入模式
    MyI2C_Delay();

    for(i = 0; i < 8; ++i)
    {
        MyI2C_W_SCL(1); // 拉高 SCL，数据被发送方输出
        MyI2C_Delay();

        if(MyI2C_R_SDA())
        {
            received_byte |= (0x80U >> i); // 读取当前位
        }

        MyI2C_W_SCL(0); // 拉低 SCL，准备接收下一位
        MyI2C_Delay();
    }
    return received_byte;
}


/**
 * @brief I2C 发送 ACK 或 NACK
 * @param AckBit 0表示发送ACK，1表示发送NACK
 */
void MyI2C_SendAck(uint8_t AckBit)
{
    MyI2C_W_SDA(AckBit);
    MyI2C_Delay();

    MyI2C_W_SCL(1); // 拉高 SCL，发送 ACK/NACK
    MyI2C_Delay();
    MyI2C_W_SCL(0); // 拉低 SCL，完成 ACK/NACK 发送
    MyI2C_Delay();

    // 发送完 ACK/NACK 后，主机释放 SDA 线，准备下一次通信
    MyI2C_W_SDA(1); // 释放 SDA，进入输入模式
}

/**
 * @brief I2C 接收 ACK 位
 * @retval 0表示接收到ACK，1表示接收到NACK
 */
uint8_t MyI2C_ReceiveAck(void)
{
    uint8_t ack_bit;

    // 主机释放 SDA 线，准备接收 ACK/NACK
    MyI2C_W_SDA(1); // 释放 SDA，进入输入模式
    MyI2C_Delay();

    MyI2C_W_SCL(1); // 拉高 SCL，发送方输出 ACK/NACK
    MyI2C_Delay();

    ack_bit = MyI2C_R_SDA(); // 读取 ACK/NACK 位

    MyI2C_W_SCL(0); // 拉低 SCL，完成 ACK/NACK 接收
    MyI2C_Delay();

    return ack_bit;
}


