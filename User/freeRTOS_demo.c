#include "freeRTOS_demo.h"

#include "FreeRTOS.h"
#include "task.h"
#include "modules_remote.h"
#include "modules_flight_input.h"
#include "modules_safety.h"
#include "modules_flight_status.h"
#include "arm_detector.h"
#include "flight_control.h"
#include "mixer.h"
#include "mpu6050.h"
#include "imu.h"
#include "debug.h"
/**************************************************************************/
/* freeRTOS 配置 */

#define START_PRIO         1
#define START_STK_SIZE   128
#define REMOTE_TASK_PRIO    2
#define REMOTE_TASK_STACK 256
#define FLIGHT_TASK_PRIO    3
#define FLIGHT_TASK_STACK 256

static TaskHandle_t StartTask_Handler = NULL;
static void StartTaskCreate(void *parameter);

static TaskHandle_t RemoteTask_Handle = NULL;
static void RemoteTask(void *parameter);

static TaskHandle_t FlightTask_Handle = NULL;
static void FlightTask(void *parameter);

/**************************************************************************/

/**********************************************************
* @funcName 
* @brief    
* @param    
* @retval   
* @details  
* @fn       
************************************************************/
void freeRTOS_demo(void)
{
    BaseType_t xReturn = pdPASS;

    xReturn = xTaskCreate(
                         (TaskFunction_t )StartTaskCreate,
                         (const char*    )"StartTaskCreate",
                         (uint32_t       )START_STK_SIZE,
                         (void*          )NULL,
                         (UBaseType_t    )START_PRIO,
                         (TaskHandle_t*  )&StartTask_Handler);

    if(xReturn == pdPASS)
    {
        LOG_I("start task create success!");
    }
    else
    {
       LOG_I("start task create failed!");
    }

    vTaskStartScheduler();
}

static void StartTaskCreate(void* parameter)
{
    BaseType_t xReturn = pdPASS;
    (void)parameter;

    Remote_Init();
    FlightState_Init();
    ArmDetector_Init();
    FlightControl_Init();

    xReturn = xTaskCreate(
                         (TaskFunction_t )RemoteTask,
                         (const char*    )"RemoteTask",
                         (uint32_t       )REMOTE_TASK_STACK,
                         (void*          )NULL,
                         (UBaseType_t    )REMOTE_TASK_PRIO,
                         (TaskHandle_t*  )&RemoteTask_Handle);
    if(xReturn != pdPASS)
    {
        while(1);
    }
    else
    {
        LOG_E("create RemoteTask successfully");
    }

    xReturn = xTaskCreate(
                         (TaskFunction_t )FlightTask,
                         (const char*    )"FlightTask",
                         (uint32_t       )FLIGHT_TASK_STACK,
                         (void*          )NULL,
                         (UBaseType_t    )FLIGHT_TASK_PRIO,
                         (TaskHandle_t*  )&FlightTask_Handle);
    if(xReturn != pdPASS)
    {
        while(1);
    }

    vTaskDelete(NULL);
}

static void RemoteTask(void *parameter)
{
    (void)parameter;
    Remote_Task(NULL);
}

static void FlightTask(void *parameter)
{
    const TickType_t desired_period = pdMS_TO_TICKS(3);
    const TickType_t control_period = (desired_period == 0U) ? 1U : desired_period;
    TickType_t last_wake = xTaskGetTickCount();
    const float dt = 0.003f;

    _st_Mpu mpu;
    _st_AngE angle;
    RemoteData_t remote = {1500U, 1500U, 1000U, 1500U, 1000U, 1000U, 1000U, 1000U, 0U, 1U};
    FlightCommand_t cmd = {0};
    SafetyStatus_t safety = {0};
    ControlOutput_t ctrl = {0};
    MotorOutput_t motor = {0};
    static uint8_t count = 0;
    static uint8_t is_first_running = 0U;

    (void)parameter;

    for (;;)
    {
        vTaskDelayUntil(&last_wake, control_period);

        if (MpuGetData() != MPU6050_OK || get_MPU6050_datas(&mpu) != MPU6050_OK)
        {
            remote = (RemoteData_t){1500U, 1500U, 1000U, 1500U, 1000U, 1000U, 1000U, 1000U, 0U, 1U};
            FlightInput_Update(&remote, &cmd);
            cmd.arm_request = ArmDetector_GetArmRequest();
            cmd.disarm_request = ArmDetector_GetDisarmRequest();
            Safety_Update(&remote, NULL, NULL, &safety);
            FlightState_Update(&cmd, &safety);
            motor_stop_all();
            continue;
        }

        if(is_first_running == 0U)
        {
            GetAngle(&mpu, &angle, dt);
            set_MPU6050_Angle(&angle);
            get_MPU6050_Angle(&angle);
            is_first_running = 1U;
        }

        count++;
        if(count >= 2)
        {
            count = 0;
            GetAngle(&mpu, &angle, dt);
            set_MPU6050_Angle(&angle);
            get_MPU6050_Angle(&angle);
        }

        if (Remote_GetLatest(&remote, 0U) != pdTRUE)
        {
            remote = (RemoteData_t){1500U, 1500U, 1000U, 1500U, 1000U, 1000U, 1000U, 1000U, 0U, 1U};
        }

        FlightInput_Update(&remote, &cmd);
        ArmDetector_Update(&remote, 3U);
        cmd.arm_request = ArmDetector_GetArmRequest();
        cmd.disarm_request = ArmDetector_GetDisarmRequest();

        // if (get_MPU6050_Angle(&angle) != MPU6050_OK)
        // {
        //     GetAngle(&mpu, &angle, dt);
        //     (void)set_MPU6050_Angle(&angle);
        // }


        Safety_Update(&remote, &mpu, &angle, &safety);
        FlightState_Update(&cmd, &safety);

        FlightControl_Update(&mpu, &angle, &cmd, FlightState_Get(), dt, &ctrl);
        Mixer_QuadX(cmd.throttle, &ctrl, FlightState_Get(), &motor);
        //motor_write_all(&motor); // 测试，暂时取消掉
    }
}
