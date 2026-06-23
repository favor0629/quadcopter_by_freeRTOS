#include "modules_remote.h"
#include "nrf24l01.h"
#include <string.h>
#include "debug.h"


#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1

#define REMOTE_PACKET_SIZE 32U      // remote pacekt size in bytes
#define REMOTE_LOST_TIMEOUT_MS 150U  // 超过这个时间没有有效帧就进入 failsafe
#define REMOTE_QUEUE_LENGTH 1U

#define REMOTE_FAULT_TOLERANT_TIME      50U     /* 允许出现短暂失联的时间，防止remote数据出现偶尔跳变 */

#define REMOTE_TASK_PERIOD      10U

/*使用的是“只保留最新值”的策略，队列长度固定为 1*/
static StaticQueue_t s_remote_queue_buffer;
static uint8_t s_remote_queue_storage[REMOTE_QUEUE_LENGTH * sizeof(RemoteData_t)];
static QueueHandle_t s_remote_queue = NULL;


/* 内部缓存：保留最新一帧 */
static RemoteData_t s_remote_cache;

/* 保留失联之前，最新的一帧有效数据，防止偶尔失联造成数据突变 */
static RemoteData_t s_remote_last_valid_cache;

static uint8_t rx_datas[REMOTE_PACKET_SIZE];

/* 最近一次收到有效遥控帧的时间 */
static TickType_t s_last_valid_tick = 0U;

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
    Remote_SetSafeState(&s_remote_last_valid_cache);
    s_last_valid_tick = xTaskGetTickCount() - pdMS_TO_TICKS(REMOTE_LOST_TIMEOUT_MS + 1U);

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
    TickType_t now_tick;

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
            /* 正确获取一帧数据，更新到s_remote_last_valid_cache */
            s_remote_last_valid_cache = new_remote;
            s_last_valid_tick = xTaskGetTickCount();
            LOG_I("[remote]Received valid remote data\r\n");
        }
        else
        {
            new_remote = s_remote_last_valid_cache;
            LOG_W("[remote]Received invalid remote data\r\n");
        }
    }
    else
    {
        /* 超时或接收失败 */
        new_remote = s_remote_last_valid_cache;
        LOG_W("[remote]Failed to receive remote data\r\n");
    }

    now_tick = xTaskGetTickCount();
    TickType_t during = now_tick - s_last_valid_tick;
    //if(during > pdMS_TO_TICKS(REMOTE_FAULT_TOLERANT_TIME))
    if( during > pdMS_TO_TICKS(REMOTE_LOST_TIMEOUT_MS))
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
    // LOG_E("[remote]failsafe=%u, elapsed=%u ms\r\n",
    //       s_remote_cache.failsafe,
    //       (uint32_t)((now_tick - s_last_valid_tick) * portTICK_PERIOD_MS));

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

   // TickType_t start_time, end_time;

    for (;;)
    {
        //start_time = xTaskGetTickCount();
        //LOG_I("Remote Task running now -------\r\n");
        Remote_Update();
        //end_time = xTaskGetTickCount() - start_time;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(REMOTE_TASK_PERIOD));


        #if (DEBUG_STACK_WATER_MARK == 1)
        UBaseType_t hw = uxTaskGetStackHighWaterMark(NULL);
        LOG_I("[remote]stack water mark:%u\r\n", hw);
        #endif 

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


