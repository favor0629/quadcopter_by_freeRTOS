#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "freeRTOS_demo.h"
#include "init.h"
#include "debug.h"

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    ALL_Init();
    //USART1_Config(9600);

    uint8_t ch = 1;

    for(uint16_t i = 0; i < 1000; ++i)
    {
        USART1_SendRawByte((uint8_t)i);
    }
    













        /* 测试裸发送 */
        
    Debug_SendRawBuffer((const uint8_t *)"TEST\r\n", 6);
    LOG_I("now we are in main fucntion");
    freeRTOS_demo();





    while(1)
    {

    }
}
