#include "modules_remote.h"
#include "nrf24l01.h"
#include <string.h>
#include "debug.h"


#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1

#define REMOTE_PACKET_SIZE 32U      // remote pacekt size in bytes
#define REMOTE_LOST_LIMIT 300U      // 连续丢包超过这个数量就进入 failsafe
#define REMOTE_QUEUE_LENGTH 1U

/*使用的是“只保留最新值”的策略，队列长度固定为 1*/
static StaticQueue_t s_remote_queue_buffer;
static uint8_t s_remote_queue_storage[REMOTE_QUEUE_LENGTH * sizeof(RemoteData_t)];
static QueueHandle_t s_remote_queue = NULL;


/* 内部缓存：保留最新一帧 */
static RemoteData_t s_remote_cache;

static uint8_t rx_datas[REMOTE_PACKET_SIZE];

/* 丢包次数统计 */
static uint16_t s_lost_count = 0U;

/* 模块是否初始化 */
static volatile uint8_t s_initialized = 0U;

static uint8_t get_is_init()
{
    uint8_t ret = 0U;
    taskENTER_CRITICAL();
    ret = s_initialized;
    taskEXIT_CRITICAL();
    return ret;
}

static void set_is_init()
{
    taskENTER_CRITICAL();
    s_initialized = 1U;
    taskEXIT_CRITICAL();
}


/* ------------------------------------------------------------
 * 安全状态
 * ------------------------------------------------------------ */
static void Remote_SetSafeState(RemoteData_t *remote)
{
    if(remote == NULL)
    {
        return;
    }

    /* 进行初始赋值 */
    remote->roll = 1500U;
    remote->pitch = 1500U;
    remote->yaw = 1500U;
    remote->throttle = 1000U;

    remote->aux1 = 1000U;
    remote->aux2 = 1000U;
    remote->aux3 = 1000U;
    remote->aux4 = 1000U;

    remote->valid = 0U;
    remote->failsafe = 1U;

}


/* ------------------------------------------------------------
 * 初始化
 * ------------------------------------------------------------ */
void Remote_Init(void)
{
    if(get_is_init() == 1U)
    {
        return;
    }

    memset(rx_datas, 0, sizeof(rx_datas));

    Remote_SetSafeState(&s_remote_cache);
    s_lost_count = REMOTE_LOST_LIMIT + 1U;

    /* 创建最新遥控帧队列，长度为1 */
    if(s_remote_queue == NULL)
    {
        s_remote_queue = xQueueCreateStatic(REMOTE_QUEUE_LENGTH,
                                    sizeof(RemoteData_t),
                                    s_remote_queue_storage,
                                    &s_remote_queue_buffer);
    }

    /* 创建失败了 */
    if(s_remote_queue == NULL)
    {
        // 失败处理
        while (1)
        {
            /* code */
        }
        
    }

    /* 把初始数据放到队列里面*/
    (void)xQueueOverwrite(s_remote_queue, &s_remote_cache);

    set_is_init();
                                    
}

/* ------------------------------------------------------------
 * 解析一帧数据
 * 返回：1 成功，0 失败
 * ------------------------------------------------------------ */
static uint8_t Remote_ParseFrame(const uint8_t *buf, RemoteData_t *out)
{
    uint8_t i;
    uint8_t check_sum = 0U;

    if(buf == NULL || out == NULL)
    {
        return 0U;
    }

    for(i = 0U; i < REMOTE_PACKET_SIZE - 1U; ++i)
    {
        check_sum += buf[i];
    }

    /* 协议校验：包头 + 校验和 */
    if(buf[0] != 0xAAU || buf[1] != 0xAFU)
    {
        return 0U;
    }

    if(buf[REMOTE_PACKET_SIZE - 1] != check_sum)
    {
        return 0U;
    }

    /* 解析摇杆数据成功 */
    out->roll     = (uint16_t)(((uint16_t)buf[4]  << 8) | buf[5]);
    out->pitch    = (uint16_t)(((uint16_t)buf[6]  << 8) | buf[7]);
    out->throttle = (uint16_t)(((uint16_t)buf[8]  << 8) | buf[9]);
    out->yaw      = (uint16_t)(((uint16_t)buf[10] << 8) | buf[11]);

    out->aux1     = (uint16_t)(((uint16_t)buf[12] << 8) | buf[13]);
    out->aux2     = (uint16_t)(((uint16_t)buf[14] << 8) | buf[15]);
    out->aux3     = (uint16_t)(((uint16_t)buf[16] << 8) | buf[17]);
    out->aux4     = (uint16_t)(((uint16_t)buf[18] << 8) | buf[19]);

    out->valid = 1U;
    out->failsafe = 0U;

    return 1U;
}

/* ------------------------------------------------------------
 * 更新一次遥控状态
 * ------------------------------------------------------------ */
void Remote_Update(void)
{
    RemoteData_t new_remote;

    /* 默认先进入安全状态，只有解析成功之后才覆盖 */
    Remote_SetSafeState(&new_remote);


    /*
     * 推荐使用阻塞式接收：
     *   NRF24L01_WaitRxPacket(rx_datas, pdMS_TO_TICKS(10))
     *
     * 如果你暂时只有 NRF24L01_RxPacket，也可以先替换成它。
     */
    if(NRF24L01_RxPacket(rx_datas) == NRF24L01_OK)
    {
        if(Remote_ParseFrame(rx_datas, &new_remote) == 1U)
        {
            s_lost_count = 0U;
            LOG_I("Received valid remote data\r\n");
        }
        else
        {
            /* 收到包但解析失败 */
            if(s_lost_count < 0xFFFFU)
            {
                s_lost_count++;
            }
            LOG_W("Received invalid remote data\r\n");
        }
    }
    else
    {
        /* 超时或接受失败 */
        if(s_lost_count < 0xFFFFU)
        {
            s_lost_count++;
        }
        LOG_W("Failed to receive remote data\r\n");
    }

    if(s_lost_count > REMOTE_LOST_LIMIT)
    {
        new_remote.valid = 0U;
        new_remote.failsafe = 1U;
    }
    else if(new_remote.valid == 0U)
    {
        new_remote.failsafe = 0U;
    }
    
    /* 更新内部缓存 */
    s_remote_cache = new_remote;
    LOG_E("lost count:%d\r\n", s_lost_count);

    (void)xQueueOverwrite(s_remote_queue, &s_remote_cache);
}

/* ------------------------------------------------------------
 * 遥控接收任务
 * ------------------------------------------------------------ */
void Remote_Task(void *argument)
{
    (void)argument;

    if(get_is_init() == 0U)
    {
        Remote_Init();
    }
    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        //LOG_I("Remote Task running now -------\r\n");
        Remote_Update();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5));
        //vTaskDelay(pdMS_TO_TICKS(5));

        /*
         * 如果你使用的 NRF24L01_WaitRxPacket 已经阻塞等待 IRQ，
         * 这里通常不需要额外延时。
         *
         * 如果你后面改回非阻塞接收，那这里要加 vTaskDelay(pdMS_TO_TICKS(1));
         */
    }
}

/* ------------------------------------------------------------
 * 获取最新遥控数据快照
 * 这里用 peek，不会把队列里的最新值取走
 * ------------------------------------------------------------ */
BaseType_t Remote_GetLatest(RemoteData_t *out, TickType_t timeout)
{
    if(out == NULL || s_remote_queue == NULL)
    {
        return pdFALSE;
    }

    return xQueuePeek(s_remote_queue, out, timeout);
}

/* ------------------------------------------------------------
 * 是否失联
 * ------------------------------------------------------------ */
uint8_t Remote_IsFailsafe(void)
{
    uint8_t fs;

    taskENTER_CRITICAL();
    fs = s_remote_cache.failsafe;
    taskEXIT_CRITICAL();

    return fs;
}









// /**
//  * @brief 初始化遥控模块
//  * @note` 1. 清零 remote 数据结构，确保所有字段有定义的初始值
//  *       2. 设置 remote 的默认值，确保在没有接收到数据时有合理的默认状态
//  */
// void Remote_Init(void)
// {
//     memset(&remote, 0, sizeof(remote));     // 初始化 remote 数据结构，确保所有字段有定义的初始值
//     memset(rx_datas, 0, sizeof(rx_datas));


//     /* 初始化遥控器数据 */
//     remote.roll = 1500;
//     remote.pitch = 1500;
//     remote.throttle = 1000;
//     remote.yaw = 1500;
//     remote.aux1 = 1000;
//     remote.aux2 = 1000;
//     remote.aux3 = 1000;
//     remote.aux4 = 1000;
//     remote.valid = 0;
//     remote.failsafe = 1;        // 无效且失联的数据
// }

// /**
//  * @brief 更新遥控数据
//  * @note 1. 调用 NRF24L01_RxPacket 接收数据包
//  *       2. 验证数据包的有效性（起始字节、校验和等）
//  *       3. 如果数据包有效，解析遥控通道数据并更新 remote 结构体
//  *       4. 如果数据包无效，增加丢包计数器，并根据丢包数量更新 failsafe 状态
//  */
// void Remote_Update(void)
// {
//     static uint16_t lost_count = REMOTE_LOST_LIMIT + 1U;
//     uint8_t i;
//     uint8_t checksum = 0;

//     if(NRF24L01_RxPacket(rx_datas) == NRF24L01_OK)
//     {
//         for(i = 0; i < REMOTE_PACKET_SIZE - 1; ++i)
//         {
//             checksum += rx_datas[i];
//         }

//         if(rx_datas[REMOTE_PACKET_SIZE - 1] == checksum && rx_datas[0] == 0xAA && rx_datas[1] == 0xAF)
//         {
//             remote.roll = (uint16_t)(((uint16_t)rx_datas[4] << 8) | rx_datas[5]);
//             remote.pitch = (uint16_t)(((uint16_t)rx_datas[6] << 8) | rx_datas[7]);
//             remote.throttle = (uint16_t)(((uint16_t)rx_datas[8] << 8) | rx_datas[9]);
//             remote.yaw = (uint16_t)(((uint16_t)rx_datas[10] << 8) | rx_datas[11]);
//             remote.aux1 = (uint16_t)(((uint16_t)rx_datas[12] << 8) | rx_datas[13]);
//             remote.aux2 = (uint16_t)(((uint16_t)rx_datas[14] << 8) | rx_datas[15]);
//             remote.aux3 = (uint16_t)(((uint16_t)rx_datas[16] << 8) | rx_datas[17]);
//             remote.aux4 = (uint16_t)(((uint16_t)rx_datas[18] << 8) | rx_datas[19]);

//             lost_count = 0;
//             remote.valid = 1;
//             remote.failsafe = 0;
//             return;
//         }
//     }

//     /*进入下面代码，则为失败*/
//     if(lost_count < 0xFFFFU)
//     {
//         lost_count++;
//     }

//     remote.valid = 0;
//     if(lost_count > REMOTE_LOST_LIMIT)
//     {
//         remote.failsafe = 1;
//     }
// }

// /**
//  * @brief 获取遥控数据
//  * @param out 输出参数，指向 RemoteData_t 结构体
//  */
// void Remote_GetData(RemoteData_t *out)
// {
//     if(out == 0)
//     {
//         return;
//     }

//     *out = remote;
// }

// /**
//  * @brief 检查遥控是否进入失联状态
//  * @return 1 如果进入失联状态，0 否则
//  */
// uint8_t Remote_IsFailsafe(void)
// {
//     return remote.failsafe;
// }
