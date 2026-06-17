#ifndef __FLASH_H
#define __FLASH_H

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <stdint.h>

/* ============================================================
 * Flash storage layout
 * 你现在定义的是单页存储区域：
 *   start = 0x08007C00
 *   end   = 0x08008000
 * 一共 1024 字节，也就是 512 个 int16_t
 * ============================================================ */
#define FLASH_Page_Size        (0x00000400UL)      /* 1KB */
#define FLASH_Start_Addr       (0x08007C00UL)
#define FLASH_End_Addr         (0x08008000UL)

/* 可存储的 int16_t 数量 */
#define FLASH_MAX_WORDS        (FLASH_Page_Size / sizeof(int16_t))

/* 简单返回状态 */
typedef enum
{
    FLASH_DRV_OK = 0,
    FLASH_DRV_ERR_PARAM,
    FLASH_DRV_ERR_RANGE,
    FLASH_DRV_ERR_MUTEX,
    FLASH_DRV_ERR_ERASE,
    FLASH_DRV_ERR_PROGRAM,
    FLASH_DRV_ERR_NOT_INIT
} FLASH_DrvStatus_t;

/* 初始化：创建互斥锁 */
void FLASH_Init(void);

/* 读 Flash：从 FLASH_Start_Addr 开始读 len 个 int16_t */
FLASH_DrvStatus_t FLASH_read(int16_t *data, uint16_t len);

/* 写 Flash：先擦除页，再从 FLASH_Start_Addr 开始写 len 个 int16_t */
FLASH_DrvStatus_t FLASH_write(const int16_t *data, uint16_t len);

#endif  /* __FLASH_H */