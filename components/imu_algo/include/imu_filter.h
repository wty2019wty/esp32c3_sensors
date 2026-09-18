/*
 * IMU 二阶低通滤波（移植自 stm32f103/mpu_filter + IMU_FilterPortable）
 *
 * 每个加速度/陀螺轴独立一个 biquad 低通，warmup 期内直通原始值。
 * 数据类型使用 mpu9250_sample_t；磁力计字段不滤波，原样透传。
 */
#pragma once

#include <stdint.h>

#include "mpu9250.h"

#define IMU_FILTER_PI 3.14159265358979323846f

typedef struct {
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
} imu_biquad_coeff_t;

typedef struct {
    float x1;
    float x2;
    float y1;
    float y2;
    uint16_t warmup_count;
    uint16_t warmup_limit;
} imu_biquad_state_t;

/* 加速度三轴 + 陀螺三轴滤波器组 */
typedef struct {
    imu_biquad_state_t accel_state[3];
    imu_biquad_coeff_t accel_coeff[3];
    imu_biquad_state_t gyro_state[3];
    imu_biquad_coeff_t gyro_coeff[3];
} imu_filter_group_t;

/**
 * @brief 初始化六轴滤波器组并设计双二阶低通系数
 *
 * @param filter          滤波器组
 * @param sample_hz       采样率（Hz），例如 100
 * @param acc_cutoff_hz   加速度截止频率（Hz）
 * @param gyro_cutoff_hz  陀螺截止频率（Hz）
 * @param warmup_limit    预热帧数：此前直接输出输入值
 */
void imu_filter_init(imu_filter_group_t *filter,
                     float sample_hz,
                     float acc_cutoff_hz,
                     float gyro_cutoff_hz,
                     uint16_t warmup_limit);

/**
 * @brief 对一帧 MPU 数据做六轴低通滤波
 *
 * @param input   原始/待滤波样本
 * @param output  滤波结果（可与 input 同址）
 * @param filter  滤波器组
 */
void imu_filter_process(const mpu9250_sample_t *input,
                        mpu9250_sample_t *output,
                        imu_filter_group_t *filter);
