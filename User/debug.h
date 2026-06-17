#ifndef __DEBUG_H
#define __DEBUG_H

#include "stm32f10x.h"
#include "stdarg.h"
#include "stdio.h"
#include "stdint.h"
#include "usart.h"
#include <string.h>


#ifdef __cplusplus
extern "C" {
#endif

/* =========================
 * 调试开关
 * ========================= */
#ifndef DEBUG_ENABLE
#define DEBUG_ENABLE                1
#endif

/* =========================
 * 调试等级
 * ========================= */
#define DEBUG_LEVEL_NONE            0
#define DEBUG_LEVEL_ERROR           1
#define DEBUG_LEVEL_WARN            2
#define DEBUG_LEVEL_INFO            3
#define DEBUG_LEVEL_DEBUG           4

#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL                 DEBUG_LEVEL_DEBUG
#endif

/* =========================
 * printf 缓冲区大小
 * 说明：
 * 1. 越大越占栈
 * 2. 太小会导致输出被截断
 * 3. 256 字节通常够用
 * ========================= */
#ifndef DEBUG_PRINTF_BUFFER_SIZE
#define DEBUG_PRINTF_BUFFER_SIZE    256U
#endif

/* =========================
 * 是否允许在调度器未启动前直接输出
 * 1: 允许，适合 main() / 初始化阶段打印
 * 0: 只允许在 FreeRTOS 正常运行后输出
 * ========================= */
#ifndef DEBUG_ALLOW_BOOT_PRINT
#define DEBUG_ALLOW_BOOT_PRINT      1
#endif

/* =========================
 * 底层输出接口
 * ========================= */
void Debug_SendChar(char ch);
void Debug_SendString(const char *str);
void Debug_Printf(const char *fmt, ...);
void Debug_SendRawBuffer(const uint8_t *data, uint16_t len);
/* 如果需要更底层控制，也可以直接调用 */
void Debug_VPrintf(const char *fmt, va_list args);

/* =========================
 * 日志宏
 * ========================= */
#if DEBUG_ENABLE

    #if (DEBUG_LEVEL >= DEBUG_LEVEL_ERROR)
        #define LOG_E(...)      Debug_Printf("[E] " __VA_ARGS__)
    #else
        #define LOG_E(...)      do {} while (0)
    #endif

    #if (DEBUG_LEVEL >= DEBUG_LEVEL_WARN)
        #define LOG_W(...)      Debug_Printf("[W] " __VA_ARGS__)
    #else
        #define LOG_W(...)      do {} while (0)
    #endif

    #if (DEBUG_LEVEL >= DEBUG_LEVEL_INFO)
        #define LOG_I(...)      Debug_Printf("[I] " __VA_ARGS__)
    #else
        #define LOG_I(...)      do {} while (0)
    #endif

    #if (DEBUG_LEVEL >= DEBUG_LEVEL_DEBUG)
        #define LOG_D(...)      Debug_Printf("[D] " __VA_ARGS__)
    #else
        #define LOG_D(...)      do {} while (0)
    #endif

#else
    #define LOG_E(...)          do {} while (0)
    #define LOG_W(...)          do {} while (0)
    #define LOG_I(...)          do {} while (0)
    #define LOG_D(...)          do {} while (0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_H */ 