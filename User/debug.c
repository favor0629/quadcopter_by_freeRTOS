#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"


/* ============================================================
 * 内部辅助函数
 * ============================================================ */

/**
 * @brief 判断当前是否处于中断上下文
 * @note  在 Cortex-M3 / CMSIS 环境下可用
 */
// static uint8_t Debug_IsInIsr(void)
// {
// //#if defined(__get_IPSR__)
//     return (__get_IPSR() != 0U) ? 1U : 0U;
// //#else
//   //  return 0U;
// //#endif
// }

static uint8_t Debug_IsInIsr(void)
{
    return ((SCB->ICSR & 0x000001FFUL) != 0U) ? 1U : 0U;
}

/**
 * @brief 判断 FreeRTOS 调度器是否已经运行
 * @note  如果工程里没有打开 INCLUDE_xTaskGetSchedulerState，则默认认为未就绪
 */
static uint8_t Debug_IsSchedulerRunning(void)
{
#if defined(INCLUDE_xTaskGetSchedulerState) && (INCLUDE_xTaskGetSchedulerState == 1)
    return (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) ? 1U : 0U;
#else
    return 0U;
#endif
}


/**
 * @brief 低层裸发送：直接轮询 USART1，不走 FreeRTOS 互斥量
 * @note
 * 1. 适合调度器未启动前
 * 2. 也可在中断里用于短文本输出
 * 3. 不适合长时间阻塞打印
 */
void Debug_SendRawBuffer(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    if(data == NULL || len == 0U)
    {
        return ;
    }

    for(i = 0; i < len; ++i)
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
        {
            /* busy wait*/
        }
        USART_SendData(USART1, data[i]);
    }
    /* wait until the last byte is fully sent */
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}


/**
 * @brief 统一发送入口
 * @note
 * - 调度器运行中：走 USART1_SendBuffer()，由你现有的 mutex 保护
 * - 调度器未启动：走裸发送，避免 mutex 未就绪
 * - 中断上下文：走裸发送，避免在 ISR 中使用 FreeRTOS mutex
 *
 * 说明：
 * 这里能保证“可用”，但如果你在 ISR 和任务里同时打日志，
 * 同一串口上的输出仍可能交错。
 * 如果你要求严格不交错，不要在 ISR 打日志。
 */
static void Debug_SendBufferSafe(const uint8_t *data, uint16_t len)
{
    if(data == NULL || len == 0U)
    {
        return;
    }

#if DEBUG_ALLOW_BOOT_PRINT
    if((Debug_IsInIsr() != 0U) || (Debug_IsSchedulerRunning()) == 0U)
    {
        Debug_SendRawBuffer(data, len);
        return;
    }
#endif
    /*调度器正常运行时，交给底层 USART 层的互斥保护*/
   // (void)USART1_SendBuffer(data, len);

   /* 即使 FreeRTOS 路径失败，也不会直接静默失效 */
    if(USART1_SendBuffer(data, len) != USART_OK)
    {
        Debug_SendRawBuffer(data, len);
    }
}


/* ============================================================
 * 对外接口
 * ============================================================ */

/**
 * @brief 发送单个字符
 */
void Debug_SendChar(char ch)
{
    uint8_t c = (uint8_t)ch;
    Debug_SendBufferSafe(&c, 1U);
}

/**
 * @brief 发送字符串
 */
void Debug_SendString(const char *str)
{
    uint16_t len;

    if(str == NULL)
    {
        return;
    }

    len = (uint16_t)strlen(str);

    if(len == 0U)
    {
        return;
    }

    Debug_SendBufferSafe((const uint8_t *)str, len);
}


/**
 * @brief 可变参数格式化输出
 * @note
 * 1. 这个函数适合在任务上下文里调用
 * 2. 不建议在中断里调用，因为 vsnprintf 本身开销较大
 * 3. 如果你要打印浮点数，newlib-nano 可能需要额外打开 float printf 支持
 */
void Debug_VPrintf(const char *fmt, va_list args)
{
#if DEBUG_ENABLE
    char buffer[DEBUG_PRINTF_BUFFER_SIZE];
    int n;

    if(fmt == NULL)
    {
        return;
    }
    n = vsnprintf(buffer, sizeof(buffer), fmt, args);
    if(n <= 0)
    {
        return;
    }
    /* vsnprintf 可能截断，buffer 仍保证有 '\0' */
    Debug_SendString(buffer);
#else   
    (void)fmt;
    (void)args;
#endif
}


/**
 * @brief printf 风格输出
 */
void Debug_Printf(const char *fmt, ...)
{
#if DEBUG_ENABLE
    va_list args;

    if(fmt == NULL)
    {
        return;
    }

    va_start(args, fmt);
    Debug_VPrintf(fmt, args);
    va_end(args);
#else
    (void)fmt;
#endif
}