/*
 * IMU 优化流水线：滤波 → 零偏补偿 → 六轴姿态
 *
 * 移植自 G:\esp32s3\stm32f103 的 Task_pm6500_Read 数据链，
 * 参数按本工程 100Hz（IMU_PERIOD_MS=10）与硬件 DLPF（陀螺 20Hz / 加速度 21Hz）调整。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "imu_attitude.h"
#include "imu_bias_calib.h"
#include "imu_filter.h"
#include "mpu9250.h"

/* 默认参数（可按应用覆盖） */
#define IMU_ALGO_DEFAULT_SAMPLE_HZ      100.0f
#define IMU_ALGO_DEFAULT_ACC_CUTOFF_HZ  20.0f
#define IMU_ALGO_DEFAULT_GYRO_CUTOFF_HZ 20.0f
#define IMU_ALGO_DEFAULT_WARMUP_FRAMES  100u    /* 100Hz 下约 1s 直通 */
#define IMU_ALGO_DEFAULT_ATT_KP         1.0f
#define IMU_ALGO_DEFAULT_ATT_KI         0.0005f

typedef struct {
    imu_filter_group_t filter;
    imu_bias_calib_t bias;
    imu_attitude_t attitude;
    float sample_dt_sec;
    bool ready;
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
 * @brief 处理一帧：滤波 → 零偏补偿 → 姿态积分
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
