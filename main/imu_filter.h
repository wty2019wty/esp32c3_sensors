/*
 * IMU 二阶低通滤波器（LP2 biquad）
 *
 * 移植自 stm32f103 工程的 IMU_FilterPortable.h / mpu_filter.c：
 *   加速度三轴、陀螺仪三轴各自独立设计一个二阶巴特沃斯低通滤波器，
 *   在进入姿态融合之前抑制高频噪声，提高陀螺仪积分的稳定性。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "mpu9250.h"

/* 二阶低通滤波器系数（直接II型传递函数：y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2） */
typedef struct {
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
} imu_filter_coeff_t;

/* 单个滤波器状态（输入/输出历史；首帧用真实采样播种，避免直通瞬态） */
typedef struct {
    float x1;
    float x2;
    float y1;
    float y2;
    bool seeded;    /* 首帧后为 true */
} imu_filter_biquad_t;

/* 滤波器组：加速度三轴 + 陀螺仪三轴各一个 */
typedef struct {
    imu_filter_biquad_t accel_state[3];
    imu_filter_coeff_t accel_coeff[3];
    imu_filter_biquad_t gyro_state[3];
    imu_filter_coeff_t gyro_coeff[3];
} imu_filter_t;

/**
 * @brief 初始化滤波器组并设计各轴系数
 *
 * @param[out] f               滤波器组
 * @param[in]  sample_hz       采样频率（Hz），应与实际 IMU 任务频率一致
 * @param[in]  accel_cutoff_hz 加速度截止频率（Hz）
 * @param[in]  gyro_cutoff_hz  陀螺仪截止频率（Hz）
 */
void imu_filter_init(imu_filter_t *f,
                     float sample_hz,
                     float accel_cutoff_hz,
                     float gyro_cutoff_hz);

/**
 * @brief 对一帧采样做滤波
 *
 * 磁力计不参与滤波，原样透传。
 *
 * @param[in]  in  原始采样
 * @param[out] out 滤波后采样（可与 in 同址）
 * @param[in,out] f 滤波器组
 */
void imu_filter_process(const mpu9250_sample_t *in,
                        mpu9250_sample_t *out,
                        imu_filter_t *f);
