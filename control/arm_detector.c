#include "arm_detector.h"
#include "FreeRTOS.h"
#include "task.h"

#ifdef DEBUG
#include "debug.h"
#endif


static ArmDetector_t arm_detector = {0};

/* ------------------------------------------------------------
 * 内部：清空状态
 * ------------------------------------------------------------ */
static void ArmDetector_ClearState(void)
{
    arm_detector.stage = ARM_STAGE_WAIT_LOW_1;
    arm_detector.timer_ms = 0U;
    arm_detector.arm_request = 0U;
    arm_detector.disarm_request = 0U;
}

// 初始化
void ArmDetector_Init(void)
{
    taskENTER_CRITICAL();
    if(arm_detector.is_initialize == 0U)
    {
        ArmDetector_ClearState();
        arm_detector.is_initialize = 1U;
    }
    taskEXIT_CRITICAL();
}

/* ------------------------------------------------------------
 * 复位
 * ------------------------------------------------------------ */
void ArmDetector_Reset(void)
{
    taskENTER_CRITICAL();
    if(arm_detector.is_initialize == 0U)
    {
        ArmDetector_ClearState();
        arm_detector.is_initialize = 1U;
    }
    else
    {
        ArmDetector_ClearState();
    }
    taskEXIT_CRITICAL();
}

/* ------------------------------------------------------------
 * 遥控无效处理
 * 失联 / 无效时直接回到初始状态，并给一个 disarm 脉冲
 * ------------------------------------------------------------ */
static void ArmDetector_HandleRemoteInvalid(void)
{
    ArmDetector_ClearState();
    arm_detector.disarm_request = 1U;
}

/* ------------------------------------------------------------
 * 更新状态机
 * ------------------------------------------------------------ */
void ArmDetector_Update(const RemoteData_t *remote, uint32_t dt_ms)
{
    if(arm_detector.is_initialize == 0U)
    {
        ArmDetector_Init();
    }

    /* 每次更新之前，先清除 */
    arm_detector.arm_request = 0U;
    arm_detector.disarm_request = 0U;

    if(remote == NULL || remote->valid == 0U || remote->failsafe != 0U)
    {
        ArmDetector_HandleRemoteInvalid();
        return;
    }

    switch(arm_detector.stage)
    {
        case ARM_STAGE_WAIT_LOW_1:
        {
            if(remote->throttle < ARM_DETECTOR_THROTTLE_LOW_TH)
            {
                arm_detector.timer_ms += dt_ms;
                if(arm_detector.timer_ms >= ARM_DETECTOR_STEP_DEBOUNCE_MS)
                {
                    arm_detector.stage = ARM_STAGE_WAIT_HIGH;
                    arm_detector.timer_ms = 0U;
                }
            }
            else
            {
                arm_detector.timer_ms = 0U;
            }
            break;
        }

        case ARM_STAGE_WAIT_HIGH:
        {
            if(remote->throttle > ARM_DETECTOR_THROTTLE_HIGH_TH)
            {
                arm_detector.timer_ms += dt_ms;
                if(arm_detector.timer_ms >= ARM_DETECTOR_STEP_DEBOUNCE_MS)
                {
                    arm_detector.stage = ARM_STAGE_WAIT_LOW_2;
                    arm_detector.timer_ms = 0U;
                }

            }
            else
            {
                arm_detector.timer_ms = 0U;
            }
            break;
        }

        case ARM_STAGE_WAIT_LOW_2:
        {
            if(remote->throttle < ARM_DETECTOR_THROTTLE_LOW_TH)
            {
                arm_detector.timer_ms += dt_ms;
                if(arm_detector.timer_ms >= ARM_DETECTOR_STEP_DEBOUNCE_MS)
                {
                    arm_detector.arm_request = 1U;
                    arm_detector.stage = ARM_STAGE_ARMED;
                    arm_detector.timer_ms = 0U;
                }
            }
            else
            {
                arm_detector.timer_ms = 0U;
            }
            break;
        }

        case ARM_STAGE_ARMED:
        {
            if(remote->throttle < ARM_DETECTOR_THROTTLE_LOW_TH)
            {
                arm_detector.timer_ms += dt_ms;
                if(arm_detector.timer_ms >= ARM_DETECTOR_DISARM_HOLD_MS)
                {
                    arm_detector.disarm_request = 1U;
                    arm_detector.stage = ARM_STAGE_WAIT_LOW_1;
                    arm_detector.timer_ms = 0U;
                }
            }
            else
            {
                arm_detector.timer_ms = 0U;
            }

            /**
             * 处于这个状态，则说明遥控需要一直有效，那么arm_request也需要一直设置为1
             */
            arm_detector.arm_request = 1U;
            break;
        }

        default:
        {
            ArmDetector_ClearState();
            break;
        }
    }
}

/* ------------------------------------------------------------
 * Getter
 * 如果只有控制任务使用，其实可以不加临界区；
 * 这里为了更稳，统一读的时候加临界区。
 * ------------------------------------------------------------ */
uint8_t ArmDetector_GetArmRequest(void)
{
    uint8_t v;
    taskENTER_CRITICAL();
    v = arm_detector.arm_request;
    taskEXIT_CRITICAL();
    return v;
}

uint8_t ArmDetector_GetDisarmRequest(void)
{
    uint8_t v;
    taskENTER_CRITICAL();
    v = arm_detector.disarm_request;
    taskEXIT_CRITICAL();
    return v;
}

ArmStage_t ArmDetector_GetStage(void)
{
    ArmStage_t s;
    taskENTER_CRITICAL();
    s = arm_detector.stage;
    taskEXIT_CRITICAL();
    return s;
}

void ArmDetector_GetSnapshot(ArmStage_t *stage,
                             uint8_t *arm_request,
                             uint8_t *disarm_request,
                             uint32_t *timer_ms)
{
    taskENTER_CRITICAL();
    if (stage != NULL)
    {
        *stage = arm_detector.stage;
    }
    if (arm_request != NULL)
    {
        *arm_request = arm_detector.arm_request;
    }
    if (disarm_request != NULL)
    {
        *disarm_request = arm_detector.disarm_request;
    }
    if (timer_ms != NULL)
    {
        *timer_ms = arm_detector.timer_ms;
    }
    taskEXIT_CRITICAL();
}

