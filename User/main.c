#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "freeRTOS_demo.h"
#include "init.h"
#include "debug.h"
#include "Delay.h"

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    Delay_Init();
    ALL_Init();
    //Delay_Init();
    
    
    Debug_SendRawBuffer((const uint8_t *)"TEST\r\n", 6);
    LOG_I("now we are in main fucntion");
    freeRTOS_demo();





    while(1)
    {

    }
}
