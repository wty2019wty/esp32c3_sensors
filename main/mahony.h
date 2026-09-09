/*
 * Mahony AHRS 姿态融合（加速度计 + 陀螺仪 + 磁力计）
 * 参考：Robert Mahony et al., "Nonlinear Complementary Filters on the
 *       Special Orthogonal Group", IEEE TAC 2008；实现基于 Madgwick 的
 *       公开参考代码（public domain）。
 */
#pragma once

#include <stdint.h>

/**
 * @brief Mahony 融合器状态（四元数 + 积分反馈）
 */
typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
    float integral_fb_x;
    float integral_fb_y;
    float integral_fb_z;
    float two_kp;   /* 比例增益 2*Kp */
    float two_ki;   /* 积分增益 2*Ki */
} mahony_t;

/**
 * @brief 初始化融合器（复位四元数并设置增益）
 *
 * @param[out] m  融合器状态
 * @param[in]  kp 比例增益（典型 0.5~1.0）
 * @param[in]  ki 积分增益（典型 0.0~0.01）
 */
void mahony_init(mahony_t *m, float kp, float ki);

/**
 * @brief 更新一次姿态解算
 *
 * @param[in,out] m   融合器状态
 * @param[in] gx,gy,gz 陀螺仪角速度（°/s）
 * @param[in] ax,ay,az 加速度计（g）
 * @param[in] mx,my,mz 磁力计（μT，全 0 时退化为六轴）
 * @param[in] dt       采样间隔（秒）
 */
void mahony_update(mahony_t *m,
                   float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float mx, float my, float mz,
                   float dt);

/**
 * @brief 取欧拉角（度）
 *
 * @param[in]  m     融合器状态
 * @param[out] roll  横滚角（°，范围 -180~180）
 * @param[out] pitch 俯仰角（°，范围 -90~90）
 * @param[out] yaw   偏航角（°，范围 -180~180）
 */
void mahony_get_euler(const mahony_t *m, float *roll, float *pitch, float *yaw);
