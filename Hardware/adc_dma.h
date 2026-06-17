#ifndef __ADC_DMA_H
#define __ADC_DMA_H

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>


/*DMA原始缓冲区长度，10*/
#define ADC_DMA_BUF_LEN 10U
#define ADC_DMA_HALF_LEN    (ADC_DMA_BUF_LEN / 2U)

/*DMA原始搬运缓冲区：DMA直接写在这个数组中*/
extern volatile uint16_t ADC_Value[ADC_DMA_BUF_LEN];

/* 平均值 */
extern volatile uint16_t ADC_AvgValue;


/**
 * @brief 初始化ADC1 + DMA1_Channel1
 * @note  需要在系统时钟配置完成后调用
 */
void ADC_DMA_Init(void);

/**
 * @brief 注册负责处理ADC数据的任务句柄
 * @note  该任务会被DMA中断用task notification唤醒
 */
void ADC_DMA_RegisterTaskHandle(TaskHandle_t task_handle);

/**
 * @brief 读取最近一次计算好的平均值
 * @return 最近一次完成的10点平均值
 */
uint16_t GetADCValue(void);

/**
 * @brief 读取最近一次完整帧的快照
 * @param[out] buf  目标缓冲区
 * @param[in]  len  目标缓冲区长度，建议传ADC_DMA_BUF_LEN
 */
void ADC_DMA_ReadLatestFrame(uint16_t *buf, uint16_t len);

/**
 * @brief ADC数据处理任务
 * @note  你需要在工程里创建这个任务
 */
void ADC_DMA_Task(void *argument);



#endif /*__ADC_DMA_H */