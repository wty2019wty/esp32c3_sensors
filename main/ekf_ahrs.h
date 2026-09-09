/*
 * EKF AHRS 姿态估计（四元数扩展卡尔曼滤波器）
 *
 * 状态向量：q = [q0, q1, q2, q3]（单位四元数）
 * 过程模型：陀螺仪角速度积分
 * 观测模型：归一化加速度计（重力方向）+ 归一化磁力计（地磁方向，可选）
 *
 * 相比 Mahony 的优势：
 *   - 协方差矩阵自适应调整滤波增益，稳态更平滑
 *   - 可独立调节过程噪声 Q 和观测噪声 R
 *   - 运动时更多信赖陀螺仪，静止时更多信赖加速度计
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief EKF AHRS 状态
 */
typedef struct {
    float q[4];                 /* 四元数 [q0, q1, q2, q3] */
    float P[4][4];              /* 误差协方差矩阵 */
    float Q_gyro;               /* 陀螺仪过程噪声方差（rad/s）^2 */
    float R_accel;              /* 加速度计观测噪声方差 */
    float R_mag;                /* 磁力计观测噪声方差（0 = 不使用磁力计） */
    bool  initialized;
} ekf_ahrs_t;

/**
 * @brief 初始化 EKF
 *
 * @param[out] ekf      EKF 状态
 * @param[in]  Q_gyro   陀螺仪过程噪声（典型 0.001~0.01）
 * @param[in]  R_accel  加速度计观测噪声（典型 0.1~1.0，越小越信任加速度计）
 * @param[in]  R_mag    磁力计观测噪声（0 表示不使用磁力计）
 */
void ekf_ahrs_init(ekf_ahrs_t *ekf, float Q_gyro, float R_accel, float R_mag);

/**
 * @brief EKF 一步预测 + 更新
 *
 * @param[in,out] ekf       EKF 状态
 * @param[in] gx,gy,gz      陀螺仪角速度（°/s）
 * @param[in] ax,ay,az      加速度计（g）
 * @param[in] mx,my,mz      磁力计（μT，全 0 时退化为六轴）
 * @param[in] dt            采样间隔（秒）
 */
void ekf_ahrs_update(ekf_ahrs_t *ekf,
                     float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float mx, float my, float mz,
                     float dt);

/**
 * @brief 取欧拉角（度）
 *
 * @param[in]  ekf     EKF 状态
 * @param[out] roll    横滚角（°，-180~180）
 * @param[out] pitch   俯仰角（°，-90~90）
 * @param[out] yaw     偏航角（°，-180~180）
 */
void ekf_ahrs_get_euler(const ekf_ahrs_t *ekf, float *roll, float *pitch, float *yaw);
