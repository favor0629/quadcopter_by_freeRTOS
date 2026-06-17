#include "adc_dma.h"
#include <string.h>

/* DMA原始数据区：DMA直接写这里 */
volatile uint16_t ADC_Value[ADC_DMA_BUF_LEN];

volatile uint16_t ADC_AvgValue = 0U;

/* 影子双缓冲：ISR把完整帧拷贝到这里，任务只读这里 */
static uint16_t s_adc_frame[2][ADC_DMA_BUF_LEN];

/* 当前正在写入的影子帧索引：0或1 */
static volatile uint8_t s_write_frame = 0U;

/* 最近一次完成的影子帧索引 */
static volatile uint8_t s_ready_frame = 0U;

/* 任务句柄 */
static TaskHandle_t s_adc_task_handle = NULL;

/* 简单的越界保护 */
static void ADC_DMA_CopyFrame(uint16_t *dst, const volatile uint16_t *src, uint16_t len)
{
    uint16_t i;
    for(i = 0U; i < len; ++i)
    {
        dst[i] = src[i];
    }
}


void ADC_DMA_RegisterTaskHandle(TaskHandle_t task_handle)
{
    s_adc_task_handle = task_handle;
}

void ADC_DMA_Init(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 时钟
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);  // 避免ADC时钟过高
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOB, ENABLE);

    /* 2) PB0 -> ADC1 Channel 8，模拟输入 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* 3) DMA1 Channel1，ADC1 -> memory */
    DMA_DeInit(DMA1_Channel1);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&ADC1->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(ADC_Value);
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = ADC_DMA_BUF_LEN;
    DMA_InitStructure.DMA_PeripheralInc = DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    /* 4) 使能DMA中断：半传输 + 传输完成 */
    DMA_ITConfig(DMA1_Channel1, DMA_IT_HT | DMA_IT_TC, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);

    /* 5) ADC1配置：单通道、连续转换 */
    ADC_DeInit(&ADC_InitStructure);
    
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_NbrOfChannel = 1;

    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_55Cycles5);

    /* ADC使用DMA */
    ADC_DMACmd(ADC1, ENABLE);

      /* 清零影子缓冲与平均值 */
    memset((void *)s_adc_frame, 0, sizeof(s_adc_frame));
    ADC_AvgValue = 0;

    /* 6) 使能ADC并校准 */
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));

    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

    /* 启动转换 */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void ADC_DMA_ReadLatestFrame(uint16_t *buf, uint16_t len)
{
    uint8_t frame;
    if(buf == NULL || len == 0U)
    {
        return;
    }

    if(len > ADC_DMA_BUF_LEN)
    {
        len = ADC_DMA_BUF_LEN;
    }

    /* 这个临界区只保护“读取影子缓冲索引 + 拷贝影子数据” */
    taskENTER_CRITICAL();
    frame = s_ready_frame;
    for(uint16_t i = 0U; i < len; ++i)
    {
        buf[i] = s_adc_frame[frame][i];
    }
    taskEXIT_CRITICAL();

}

uint16_t GetADCValue(void)
{
    return ADC_AvgValue;
}

/**
 * @brief ADC数据处理任务
 * @note  任务会被DMA1_Channel1_IRQHandler唤醒
 *        这里直接对“完整10点帧”求平均，输出到ADC_AvgValue
 */
void ADC_DMA_Task(void *argument)
{
    uint16_t local_frame[ADC_DMA_BUF_LEN];
    uint32_t sum;
    uint16_t i;

    (void)argument;

    while(1)
    {
        // 等待DMA中断通知
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 取出最近一次完整帧的快照
        ADC_DMA_ReadLatestFrame(local_frame, ADC_DMA_BUF_LEN);

        // 计算平均值
        sum = 0;
        for(i = 0U; i < ADC_DMA_BUF_LEN; ++i)
        {
            sum += local_frame[i];
        }

        ADC_AvgValue = (uint16_t)(sum / ADC_DMA_BUF_LEN);
    }
}

/**
 * @brief DMA1_Channel1中断服务函数
 * @note  半传输时复制前半段，传输完成时复制后半段，然后通知任务
 */
void DMA1_Channel1_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* 半传输：前半帧已稳定，可拷贝到影子缓冲 */
    if(DMA_GetITStatus(DMA1_IT_HT1) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_HT1);

        ADC_DMA_CopyFrame(&s_adc_frame[s_write_frame][0],
        &ADC_Value[0], ADC_DMA_HALF_LEN);

    }
    /* 传输完成：后半帧已稳定，补齐影子缓冲并通知任务 */
    if(DMA_GetITStatus(DMA1_IT_TC1) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_TC1);

        ADC_DMA_CopyFrame(&s_adc_frame[s_write_frame][ADC_DMA_HALF_LEN],
        &ADC_Value[ADC_DMA_HALF_LEN], ADC_DMA_HALF_LEN);

        /* 当前帧就绪，切换到另一块影子缓冲 */
        s_ready_frame = s_write_frame;
        s_write_frame ^= 1U;

        /* 唤醒处理任务*/
        if(s_adc_task_handle != NULL)
        {
            vTaskNotifyGiveFromISR(s_adc_task_handle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

    }

}