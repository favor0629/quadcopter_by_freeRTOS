#include "led_control.h"
#include "FreeRTOS.h"
#include "task.h"
#include "led.h"
#include "debug.h"
#include "modules_flight_status.h"


void LED_ControlTask(void *parameter)
{
    (void)parameter;
    LED_status_e led_status;
    FlightState_t flight_status;
    TickType_t xLastWakeTime; 
    TickType_t cycle = pdMS_TO_TICKS(3);

    if(LED_GetIsInitialized()== 0U)
    {
        LOG_W("led don't initilized!\r\n");
    }
    xLastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, cycle);
        flight_status = FlightState_Get();
        switch (flight_status)
        {
            case FLIGHT_LOCKED:
            {
                //LED_SetStatus()
                led_status = AlwaysOn;
                break;
            }

            case FLIGHT_ARMED_IDLE:
            {
                led_status = AlwaysOff;
                break;
            }
            
            case FLIGHT_FLYING:
            {
                led_status = AlwaysOff;
                break;
            }

            case FLIGHT_FAILSAFE:
            {
                led_status = AllFlashLight;
                break;
            }

            case FLIGHT_ERROR:
            {
                led_status = DANGEROURS;
                break;
            }

            default:
                led_status = AlwaysOn;
                break;
        }
        
        LED_SetStatus(led_status);
        LOG_I("led status:%d, flight_status:%d\r\n", led_status, flight_status);
    }
}