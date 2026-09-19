/*
 * IMU 优化流水线：滤波 → 零偏补偿 → 六轴姿态
 *
 * 时序（200Hz，见 imu_bias_calib.c）：
 *   1) 上电 delay 1000 帧 ≈ 5s：陀螺预热，滤波 warmup，不积分姿态
 *   2) 静止校准 400 帧 ≈ 2s：估 gyro/accel 零偏，仍不积分姿态
 *   3) 校准完成瞬间：姿态归零（当前姿态作为 R/P/Y 原点）
 *   4) 之后：零偏补偿数据进入 Mahony 积分
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "imu_attitude.h"
#include "imu_bias_calib.h"
#include "imu_filter.h"
#include "mpu9250.h"

/* 默认参数：按 200Hz 采样设计（与 IMU_PERIOD_MS=5 对齐）
 * 截止频率略高于 100Hz 方案，仍落在硬件 DLPF（陀螺 20Hz / 加速度 21Hz）附近；
 * 滤波 warmup 200 帧 ≈ 1s 直通；零偏 delay+calib 共 1400 帧 ≈ 7s。 */
#define IMU_ALGO_DEFAULT_SAMPLE_HZ      200.0f
#define IMU_ALGO_DEFAULT_ACC_CUTOFF_HZ  25.0f
#define IMU_ALGO_DEFAULT_GYRO_CUTOFF_HZ 25.0f
#define IMU_ALGO_DEFAULT_WARMUP_FRAMES  200u
#define IMU_ALGO_DEFAULT_ATT_KP         1.0f
#define IMU_ALGO_DEFAULT_ATT_KI         0.0005f

typedef struct {
    imu_filter_group_t filter;
    imu_bias_calib_t bias;
    imu_attitude_t attitude;
    float sample_dt_sec;
    bool ready;
    bool attitude_started; /* 校准完成后已归零并开始积分 */
} imu_pipeline_t;

/**
 * @brief 按默认参数初始化整条流水线
 *
 * @param pipeline    流水线状态
 * @param sample_hz   实际采样率（Hz），需与 IMU 任务周期一致
 */
void imu_pipeline_init_defaults(imu_pipeline_t *pipeline, float sample_hz);

/**
 * @brief 完整参数初始化
 */
void imu_pipeline_init(imu_pipeline_t *pipeline,
                       float sample_hz,
                       float acc_cutoff_hz,
                       float gyro_cutoff_hz,
                       uint16_t warmup_frames,
                       float att_kp,
                       float att_ki);

/**
 * @brief 处理一帧：滤波 → 零偏补偿 →（校准完成后）姿态积分
 *
 * 上电预热与静止校准期间不更新姿态；校准完成首帧会将姿态归零后开始积分。
 *
 * @param pipeline 流水线
 * @param raw      驱动读出的原始物理量样本
 * @param out      最终补偿后数据（可为 NULL）
 * @return true 本帧处理成功；false 参数无效
 */
bool imu_pipeline_process(imu_pipeline_t *pipeline,
                          const mpu9250_sample_t *raw,
                          mpu9250_sample_t *out);

bool imu_pipeline_calib_done(const imu_pipeline_t *pipeline);
