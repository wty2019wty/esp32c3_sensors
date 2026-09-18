/*
 * IMU 偏置校准与静止零偏跟踪
 *
 * 移植并强化自 stm32f103 工程的 imu_bias_calib.c：
 *   1) 上电后采集若干帧“静止”数据，求陀螺仪零偏（°/s）与加速度计零偏（g）；
 *      校准期间若检测到运动则丢弃样本，避免把角速度当成零偏；
 *      持续运动或零偏过大导致长期采不到静止样本时，超时后强制结束，
 *      保证姿态融合能启动（零偏交由运行期跟踪收敛）；
 *   2) 运行期用“陀螺残差 + 帧间变化 + 加速度量级/变化”多重判据识别静止，
 *      对陀螺零偏做自适应低通跟踪，抑制温漂/零偏随时间的缓慢变化；
 *   3) 记录校准时的片内温度，并在线估计零偏温漂系数（dps/℃），
 *      在非静止无法跟踪时按温度外推补偿，减少温漂造成的积分误差；
 *   4) 暴露静止标志，供姿态融合做自适应增益与 ZUPT。
 *
 * 说明：加速度计零偏仅在“接近水平静止”时才会标定，避免设备倾斜放置时
 *       误把重力分量当成零偏而破坏姿态方向（这是相对原 STM32 实现的加固）。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "mpu9250.h"

/* IMU 偏置校准与跟踪状态 */
typedef struct {
    float gyro_bias[3];   /* 陀螺仪零偏（参考温度处），单位 °/s */
    float accel_bias[3];  /* 加速度计零偏，单位 g  */

    float calib_acc_sum[3];  /* 校准期间加速度累计 */
    float calib_gyro_sum[3]; /* 校准期间陀螺仪累计 */
    uint16_t calib_sample_cnt;  /* 已累计的静止样本数 */
    uint16_t calib_total_cnt;   /* 校准期总帧数，用于超时兜底 */

    mpu9250_sample_t prev_input; /* 上一帧输入，用于静止检测 */
    uint8_t calib_phase;         /* 0: 正在校准, 1: 校准完成并跟踪 */
    uint16_t still_cnt;          /* 连续静止帧计数 */
    bool still;                  /* 当前是否判定为静止 */

    /* 温漂补偿：ref_temp 处的 gyro_bias + tempco*(T-ref_temp) 为当前零偏 */
    float ref_temp_c;            /* 陀螺零偏对应的参考温度 ℃ */
    float gyro_tempco[3];        /* 在线估计的零偏温漂系数 °/s/℃ */
    float tempco_valid;          /* 0~1，估计置信度（越大越信） */
    float last_track_temp_c;     /* 上次仍跟踪时的温度 */
    float last_track_bias[3];    /* 上次仍跟踪后的零偏 */
    bool  tempco_ready;          /* 是否已有可用的温漂系数 */
} imu_bias_calib_t;

/**
 * @brief 初始化校准状态
 */
void imu_bias_calib_init(imu_bias_calib_t *s);

/**
 * @brief 更新偏置校准/跟踪，并输出补偿后的数据
 *
 * @param[in,out] s      校准状态
 * @param[in]     input  输入数据（应为滤波后的物理单位数据）
 * @param[out]    output 补偿后数据（可与 input 同址）
 */
void imu_bias_calib_update(imu_bias_calib_t *s,
                           const mpu9250_sample_t *input,
                           mpu9250_sample_t *output);

/**
 * @brief 校准是否已完成（true 表示已进入运行期跟踪）
 */
bool imu_bias_calib_is_done(const imu_bias_calib_t *s);

/**
 * @brief 当前是否判定为静止（连续满足静止判据达到保持帧数）
 *
 * 校准完成前恒为 false。供 Mahony 自适应增益 / ZUPT 使用。
 */
bool imu_bias_calib_is_still(const imu_bias_calib_t *s);
