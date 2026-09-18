/*
 * IMU 零偏校准与在线跟踪（移植自 stm32f103/imu_bias_calib）
 *
 * 启动阶段静止采样 N 帧，估计陀螺/加速度零偏；
 * 随后在设备近似静止时用极低系数低通跟踪陀螺零偏温漂。
 */
#pragma once

#include <stdint.h>

#include "mpu9250.h"

typedef struct {
    float gyro_bias[3];      /* °/s */
    float accel_bias[3];     /* g */

    float calib_acc_sum[3];
    float calib_gyro_sum[3];
    uint16_t calib_delay_cnt;   /* 上电等待计数（约 2s @200Hz，期间不校准） */
    uint16_t calib_sample_cnt;  /* 校准采样计数（约 2s @200Hz） */

    mpu9250_sample_t prev_input;
    uint8_t calib_phase;     /* 0: 等待/校准中, 1: 完成并进入跟踪 */
} imu_bias_calib_t;

void imu_bias_calib_init(imu_bias_calib_t *state);
uint8_t imu_bias_calib_is_done(const imu_bias_calib_t *state);

/**
 * @brief 更新零偏估计，并输出补偿后的样本
 *
 * @param state  校准状态
 * @param input  滤波后输入（物理单位 g / °/s）
 * @param output 补偿后输出（可与 input 同址）
 */
void imu_bias_calib_update(imu_bias_calib_t *state,
                           const mpu9250_sample_t *input,
                           mpu9250_sample_t *output);
