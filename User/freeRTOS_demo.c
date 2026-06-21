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
#include "led.h"

/**************************************************************************/
/* freeRTOS 配置 */

#define START_PRIO         1
#define START_STK_SIZE   128
#define REMOTE_TASK_PRIO    3
#define REMOTE_TASK_STACK 256
#define FLIGHT_TASK_PRIO    3
#define FLIGHT_TASK_STACK 512
#define SENSOR_TASK_PRIO    4
#define SENSOR_TASK_STACK 384

/* led 任务 */
#define LED_TASK_PRIO       3
#define LED_TASK_STACK      128
static TaskHandle_t LedTask_handler = NULL;

static TaskHandle_t StartTask_Handler = NULL;
static void StartTaskCreate(void *parameter);

static TaskHandle_t RemoteTask_Handle = NULL;
static void RemoteTask(void *parameter);

static TaskHandle_t SensorTask_Handle = NULL;
static void SensorTask(void *parameter);

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
        //LOG_I("start task create success!\r\n");
       // LOG_I("")
       // 串口打印
       
    
    }
    else
    {
       //LOG_I("start task create failed!\r\n");
    }

    vTaskStartScheduler();
}

static void StartTaskCreate(void* parameter)
{
	LOG_E("start StartTaskCreate function \r\n");
    BaseType_t xReturn = pdPASS;
    (void)parameter;

    Remote_Init();
    FlightState_Init();
    ArmDetector_Init();
    FlightControl_Init();

    taskENTER_CRITICAL();
    xReturn = xTaskCreate(
                         (TaskFunction_t )RemoteTask,
                         (const char*    )"RemoteTask",
                         (uint32_t       )REMOTE_TASK_STACK,
                         (void*          )NULL,
                         (UBaseType_t    )REMOTE_TASK_PRIO,
                         (TaskHandle_t*  )&RemoteTask_Handle);
    if(xReturn != pdPASS)
    {
         LOG_E("create RemoteTask failed\r\n");
    }
    else
    {
        LOG_E("create RemoteTask successfully\r\n");
    }

    xReturn = xTaskCreate(
                         (TaskFunction_t )SensorTask,
                         (const char*    )"SensorTask",
                         (uint32_t       )SENSOR_TASK_STACK,
                         (void*          )NULL,
                         (UBaseType_t    )SENSOR_TASK_PRIO,
                         (TaskHandle_t*  )&SensorTask_Handle);
    if(xReturn != pdPASS)
    {
         LOG_E("create SensorTask failed\r\n");
    }
    else
    {
        LOG_E("create SensorTask successfully\r\n");
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
        LOG_E("create FlightTask failed\r\n");
    }
    else
    {
        LOG_E("create FlightTask successfully!\r\n");
    }

    xReturn = xTaskCreate((TaskFunction_t)LED_Task,
                         (const char*    )"LEDTask",
                         (uint32_t       )LED_TASK_STACK,
                         (void*          )NULL,
                         (UBaseType_t    )LED_TASK_PRIO,
                         (TaskHandle_t*  )&LedTask_handler);

    taskEXIT_CRITICAL();

    vTaskDelete(NULL);
}

static void RemoteTask(void *parameter)
{
    LOG_I("begin remote task\r\n ");
    (void)parameter;

    /* 这里对led进行测试 */

    //     AlwaysOn = 1,
    // AlwaysOff,
    // AllFlashLight,
    // AlternateFlash,
    // WARNING,
    // DANGEROURS,
    // GET_OFFSET,
    // BREATHING  
    LED_status_e led_status = BREATHING;
    LED_SetStatus(led_status);
    Remote_Task(NULL);
}

static void SensorTask(void *parameter)
{
    const TickType_t sensor_period = pdMS_TO_TICKS(3);
    const float angle_dt = 0.006f;
    TickType_t last_wake = xTaskGetTickCount();
    uint8_t angle_divider = 0U;
    uint8_t first_angle_update = 1U;
    _st_Mpu mpu = {0};
    _st_AngE angle = {0};

    LOG_I("begin sensor task\r\n ");
    (void)parameter;

    for (;;)
    {
        vTaskDelayUntil(&last_wake, sensor_period);

        (void)MpuGetData();
        (void)get_MPU6050_datas(&mpu);

        if (first_angle_update != 0U || angle_divider >= 1U)
        {
            GetAngle(&mpu, &angle, angle_dt);
            (void)set_MPU6050_Angle(&angle);

            first_angle_update = 0U;
            angle_divider = 0U;
        }
        else
        {
            angle_divider++;
        }
    }
}

static void FlightTask(void *parameter)
{
    LOG_I("begin flight task\r\n ");
    const TickType_t desired_period = pdMS_TO_TICKS(10);
    const TickType_t control_period = (desired_period == 0U) ? 1U : desired_period;
    TickType_t last_wake = xTaskGetTickCount();
    const float dt = 0.010f;

    uint8_t count = 0;
    _st_Mpu mpu;
    _st_AngE angle;
    RemoteData_t remote = {1500U, 1500U, 1000U, 1500U, 1000U, 1000U, 1000U, 1000U, 0U, 1U};
    FlightCommand_t cmd = {0};
    SafetyStatus_t safety = {0};
    ControlOutput_t ctrl = {0};
    MotorOutput_t motor = {0};

    (void)parameter;

    for (;;)
    {
        vTaskDelayUntil(&last_wake, control_period);

        if (get_MPU6050_datas(&mpu) != MPU6050_OK)
        {
            mpu = (_st_Mpu){0};
        }

        if (get_MPU6050_Angle(&angle) != MPU6050_OK)
        {
            angle = (_st_AngE){0};
        }

        if (Remote_GetLatest(&remote, 0U) != pdTRUE)
        {
            remote = (RemoteData_t){1500U, 1500U, 1000U, 1500U, 1000U, 1000U, 1000U, 1000U, 0U, 1U};
            LOG_E("Failed to get latest remote data\r\n");
        }
        else
        {
            // LOG_I("Got remote data: roll=%u, pitch=%u, throttle=%u, yaw=%u\r\n",
            //       remote.roll, remote.pitch, remote.throttle, remote.yaw);
        }

        FlightInput_Update(&remote, &cmd);
        // ArmDetector_Update(&remote, 3U);
        // cmd.arm_request = ArmDetector_GetArmRequest();
        // cmd.disarm_request = ArmDetector_GetDisarmRequest();



        // if (get_MPU6050_Angle(&angle) != MPU6050_OK)
        // {
        //     GetAngle(&mpu, &angle, dt);
        //     (void)set_MPU6050_Angle(&angle);
        // }
        cmd.arm_request = 1U; // 测试，暂时强制解锁
        cmd.disarm_request = 0U; // 测试，暂时不允许上锁


        Safety_Update(&remote, &mpu, &angle, &safety);
        FlightState_Update(&cmd, &safety);

        count++;
        if(count >= 5)
        {
            count = 0;
            LOG_I("safety: remote lost:%d\r\n", safety.remote_lost);
        }
        // LOG_I("================================================================\r\n");
        // LOG_I("safety status: remote_lost=%u, mpu_lost=%u, angle_too_large=%u, emergency=%u\r\n",
        //      safety.remote_lost, safety.mpu_lost, safety.angle_too_large, safety.emergency);
        // LOG_I("remote: th=%u, arm=%u, disarm=%u | angle: roll=%.2f, pitch=%.2f, yaw=%.2f | safety: remote_lost=%u, mpu_lost=%u, angle_too_large=%u, emergency=%u | state=%d\r\n",
        //       remote.throttle, cmd.arm_request, cmd.disarm_request,
        //       angle.roll, angle.pitch, angle.yaw,
        //       safety.remote_lost, safety.mpu_lost, safety.angle_too_large, safety.emergency,
        //       FlightState_Get());

        FlightControl_Update(&mpu, &angle, &cmd, FlightState_Get(), dt, &ctrl);
        Mixer_QuadX(cmd.throttle, &ctrl, FlightState_Get(), &motor);
        LOG_I("thr:%d, control output: roll=%.2f, pitch=%.2f, yaw=%.2f | motor: m1=%d, m2=%d, m3=%d, m4=%d\r\n",
              remote.throttle ,ctrl.roll_out, ctrl.pitch_out, ctrl.yaw_out,
              motor.m1, motor.m2, motor.m3, motor.m4);

        //motor_write_all(&motor); // 测试，暂时取消掉
    }
}
