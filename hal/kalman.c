#include "kalman.h"



/**
 * 6.1 Q：过程噪声协方差

表示你对“模型预测”的不确定程度。

Q 小：更相信模型，输出更平滑，但反应慢
Q 大：更不信模型，输出更灵敏，但抖动更明显
6.2 R：测量噪声协方差

表示你对“传感器测量”的不确定程度。

R 小：更相信测量值，输出跟得快
R 大：更怀疑传感器，输出更稳
6.3 P0：初始协方差

表示你对初始估计值的信心。

P0 小：表示一开始就很相信初值
P0 大：表示一开始不太相信初值，会更快跟随后续测量值

工程上常见做法：

初值直接设为第一次测量
P0 = 1.0f 或者更大一些
根据实际响应速度再调 Q/R 比例，达到既能快速跟踪又不过于抖动的效果。



 */
/* 
 * 工程建议：
 * 1. Q >= 0
 * 2. R > 0
 * 3. P >= 0
 * 
 * Q 越大，滤波器越“跟手”，但平滑性下降。
 * R 越大，越不信任测量值，输出越平滑。
 */

static float kalman_clamp_nonnegative(float v)
{
    return (v < 0.0f) ? 0.0f : v;
}

void Kalman1D_Init(Kalman1D_t *kf, float x0, float p0, float q, float r)
{
    if (kf == 0) {
        return;
    }

    kf->x = x0;

    /* 协方差必须非负 */
    kf->P = (p0 >= 0.0f) ? p0 : 1.0f;
    kf->Q = kalman_clamp_nonnegative(q);

    /* R 理论上应大于 0，避免分母为 0 */
    kf->R = (r > 0.0f) ? r : 1.0f;

    kf->initialized = 1U;
}

float Kalman1D_Update(Kalman1D_t *kf, float z)
{
    float P_pred;
    float K;
    float denom;

    if (kf == 0) {
        return 0.0f;
    }

    /* 第一次输入时，直接用测量值做初值，工程上最稳妥 */
    if (kf->initialized == 0U) {
        Kalman1D_Init(kf, z, 1.0f, kf->Q, kf->R);
        return kf->x;
    }

    /* 1) 预测协方差 */
    P_pred = kf->P + kf->Q;

    /* 2) 计算卡尔曼增益前做除零保护 */
    denom = P_pred + kf->R;
    if (denom <= 1e-12f) {
        /* 参数非法或极端异常，退化为直接输出测量值 */
        kf->x = z;
        kf->P = P_pred;
        return kf->x;
    }

    K = P_pred / denom;

    /* 3) 更新估计值 */
    kf->x = kf->x + K * (z - kf->x);

    /* 4) 更新协方差 */
    kf->P = (1.0f - K) * P_pred;

    return kf->x;
}

void Kalman1D_Reset(Kalman1D_t *kf)
{
    if (kf == 0) {
        return;
    }

    kf->x = 0.0f;
    kf->P = 1.0f;
    kf->Q = 0.0f;
    kf->R = 1.0f;
    kf->initialized = 0U;
}

