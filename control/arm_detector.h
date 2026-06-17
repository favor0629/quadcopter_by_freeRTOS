#ifndef __ARM_DETECTOR_H
#define __ARM_DETECTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "modules_remote.h"
#include "stm32f10x.h"

#ifndef ARM_DETECTOR_THROTTLE_LOW_TH
#define ARM_DETECTOR_THROTTLE_LOW_TH        1100U
#endif

#ifndef ARM_DETECTOR_THROTTLE_HIGH_TH
#define ARM_DETECTOR_THROTTLE_HIGH_TH       1750U
#endif

#ifndef ARM_DETECTOR_STEP_DEBOUNCE_MS
#define ARM_DETECTOR_STEP_DEBOUNCE_MS       100U
#endif

#ifndef ARM_DETECTOR_DISARM_HOLD_MS
#define ARM_DETECTOR_DISARM_HOLD_MS         300U
#endif

typedef enum
{
    ARM_STAGE_WAIT_LOW_1 = 0,
    ARM_STAGE_WAIT_HIGH,
    ARM_STAGE_WAIT_LOW_2,
    ARM_STAGE_ARMED
} ArmStage_t;

typedef struct
{
    ArmStage_t stage;
    uint32_t   timer_ms;

    uint8_t    arm_request;      /* 单周期脉冲 */
    uint8_t    disarm_request;   /* 单周期脉冲 */

    uint8_t    is_initialize;
} ArmDetector_t;

void ArmDetector_Init(void);
void ArmDetector_Reset(void);

/* dt_ms 建议由控制任务的固定周期提供 */
void ArmDetector_Update(const RemoteData_t *remote, uint32_t dt_ms);

uint8_t ArmDetector_GetArmRequest(void);
uint8_t ArmDetector_GetDisarmRequest(void);
ArmStage_t ArmDetector_GetStage(void);

/* 可选：一次性读取快照，避免多次 getter 之间状态变化 */
void ArmDetector_GetSnapshot(ArmStage_t *stage,
                             uint8_t *arm_request,
                             uint8_t *disarm_request,
                             uint32_t *timer_ms);

#ifdef __cplusplus
}
#endif

#endif /* __ARM_DETECTOR_H */

