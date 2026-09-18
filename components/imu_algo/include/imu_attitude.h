/*
 * 六轴姿态融合（移植自 stm32f103/attitude6axis，Mahony 互补滤波）
 *
 * 输入：补偿后的 acc(g) + gyro(°/s)，输出四元数与 roll/pitch/yaw。
 * 与 STM32 机架安装不同，本工程按标准 ZYX 欧拉角输出，不做 pitch/roll 对调。
 */
#pragma once

#include <stdint.h>

#include "mpu9250.h"

typedef struct {
    float q0, q1, q2, q3;
    float ex_int, ey_int, ez_int;
    float kp, ki;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
} imu_attitude_t;

void imu_attitude_init(imu_attitude_t *state, float kp, float ki);
void imu_attitude_reset(imu_attitude_t *state);

/**
 * @brief 用一帧 IMU 数据推进姿态解算
 *
 * @param state   姿态状态
 * @param data    偏置补偿后的样本（g / °/s）
 * @param dt_sec  本帧积分步长（秒），必须 > 0
 */
void imu_attitude_update(imu_attitude_t *state,
                         const mpu9250_sample_t *data,
                         float dt_sec);
