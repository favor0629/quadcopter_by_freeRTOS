#include "flash.h"

/* ============================================================
 * FreeRTOS mutex
 * 用于保护 Flash 读写互斥
 * ============================================================ */
//static StaticSemaphore_t s_flash_mutex_buffer;
static SemaphoreHandle_t s_flash_mutex = NULL;


/* ============================================================
* 内部工具函数
* ============================================================ */

static FLASH_DrvStatus_t FLASH_LockAccess(TickType_t wait_time)
{
    if(s_flash_mutex == NULL)
    {
        return FLASH_DRV_ERR_NOT_INIT;
    }

    if(xSemaphoreTake(s_flash_mutex, wait_time) != pdTRUE)  // 获取互斥锁
    {
        return FLASH_DRV_ERR_MUTEX;
    }

    return FLASH_DRV_OK;
}

static void FLASH_UnlockAccess(void)
{
    if(s_flash_mutex != NULL)
    {
        xSemaphoreGive(s_flash_mutex);      // 释放锁
    }
}

static FLASH_DrvStatus_t FLASH_CheckParam(const void *data, uint16_t len)
{
    if(data == NULL)
    {
        return FLASH_DRV_ERR_PARAM;     // 参数错误
    }

    if(len == 0U)
    {
        return FLASH_DRV_ERR_PARAM;
    }

    if(len > FLASH_MAX_WORDS)
    {
        return FLASH_DRV_ERR_RANGE;
    }

    return FLASH_DRV_OK;
}


/**
 * @brief 初始化 Flash 访问互斥锁
 * @note  建议在 scheduler 启动前或系统初始化阶段调用一次
 */
void FLASH_Init(void)
{
    if(s_flash_mutex == NULL)
    {
         s_flash_mutex = xSemaphoreCreateMutex();

        //xSemaphoreCreateMutexStatic()
    }
}




/**
 * @brief 将数据写入 Flash
 * @param data  源数据
 * @param len   写入长度，单位是 int16_t 个数
 * @return 状态码
 *
 * 注意：
 * 1. 这个函数必须在任务上下文中调用，不能在中断里调用。
 * 2. 该实现会先擦除整页，再按半字写入。
 * 3. len 不能超过 FLASH_MAX_WORDS。
 */
FLASH_DrvStatus_t FLASH_write(const int16_t *data, uint16_t len)
{
    FLASH_DrvStatus_t ret;
    uint32_t address;
    uint16_t i;

    ret = FLASH_CheckParam(data, len);
    if (ret != FLASH_DRV_OK)
    {
        return ret;
    }

    ret = FLASH_LockAccess(portMAX_DELAY);
    if (ret != FLASH_DRV_OK)
    {
        return ret;
    }

    /* 解锁 Flash */
    FLASH_Unlock();

    /* 清标志位 */
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    /* 擦除目标页 */
    if (FLASH_ErasePage(FLASH_Start_Addr) != FLASH_COMPLETE)
    {
        FLASH_Lock();
        FLASH_UnlockAccess();
        return FLASH_DRV_ERR_ERASE;
    }

    /* 从页首开始写 */
    address = FLASH_Start_Addr;

    for (i = 0U; i < len; i++)
    {
        if (FLASH_ProgramHalfWord(address, (uint16_t)data[i]) != FLASH_COMPLETE)
        {
            FLASH_Lock();
            FLASH_UnlockAccess();
            return FLASH_DRV_ERR_PROGRAM;
        }

        address += 2U;
    }

    /* 上锁，防止误操作 */
    FLASH_Lock();

    FLASH_UnlockAccess();
    return FLASH_DRV_OK;
}

/**
 * @brief 从 Flash 读出数据
 * @param data  目标缓冲区
 * @param len   读取长度，单位是 int16_t 个数
 * @return 状态码
 */
FLASH_DrvStatus_t FLASH_read(int16_t *data, uint16_t len)
{
    uint32_t address;
    uint16_t i;
    FLASH_DrvStatus_t ret;

    ret = FLASH_CheckParam(data, len);
    if (ret != FLASH_DRV_OK)
    {
        return ret;
    }

    ret = FLASH_LockAccess(portMAX_DELAY);
    if (ret != FLASH_DRV_OK)
    {
        return ret;
    }

    address = FLASH_Start_Addr;

    for (i = 0U; i < len; i++)
    {
        data[i] = *(__IO int16_t *)address;
        address += 2U;
    }

    FLASH_UnlockAccess();
    return FLASH_DRV_OK;
}