#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float x;            /* 当前估计值 */
    float P;            /* 当前估计协方差 */
    float Q;            /* 过程噪声协方差 */
    float R;            /* 测量噪声协方差 */
    uint8_t initialized;/* 是否已初始化 */
} Kalman1D_t;

/**
 * @brief  初始化一维卡尔曼滤波器
 * @param  kf  滤波器对象指针
 * @param  x0  初始估计值
 * @param  p0  初始协方差
 * @param  q   过程噪声协方差
 * @param  r   测量噪声协方差
 */
void Kalman1D_Init(Kalman1D_t *kf, float x0, float p0, float q, float r);

/**
 * @brief  更新一维卡尔曼滤波器
 * @param  kf  滤波器对象指针
 * @param  z   当前测量值
 * @return  更新后的估计值
 */
float Kalman1D_Update(Kalman1D_t *kf, float z);

/**
 * @brief  复位滤波器，使其回到未初始化状态
 * @param  kf  滤波器对象指针
 */
void Kalman1D_Reset(Kalman1D_t *kf);

#ifdef __cplusplus
}
#endif

#endif

// #ifndef _KALMAN_H
// #define _KALMAN_H




// struct _1_ekf_filter
// {
// 	float LastP;// 0.02
// 	float	Now_P; // 0
// 	float out; //8192
// 	float Kg; //0
// 	float Q;  // 0.001
// 	float R;	// 0.543
// };

// //void ekf_1(struct EKF *ekf,void *input);  //一维卡尔曼
// extern void kalman_1(struct _1_ekf_filter *ekf,float input);  //一维卡尔曼

// #endif


// static struct _1_ekf_filter ekf[3] = {{0.02,0,8192,0,0.001,0.543},
// {0.02,0,8192,0,0.001,0.543},{0.02,0,8192,0,0.001,0.543}};
