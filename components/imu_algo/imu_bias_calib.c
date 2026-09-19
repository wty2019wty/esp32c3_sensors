#include "imu_bias_calib.h"

#include <string.h>

/* 时序按 200Hz（IMU_PERIOD_MS=5）配置：
 *   上电 delay 1000 帧 ≈ 5s：陀螺预热 / 模块稳定，期间不累计零偏、不积分姿态
 *   然后 calib 400 帧 ≈ 2s：静止校准，仍不积分姿态
 *   之后进入在线跟踪 */
#define CALIB_DELAY_SAMPLE_NUM  1000
#define CALIB_SAMPLE_NUM        400
#define GYRO_TRACK_THRES_DPS    0.5f
#define GYRO_TRACK_THRES_SQ     (GYRO_TRACK_THRES_DPS * GYRO_TRACK_THRES_DPS)
#define TRACK_ALPHA             0.002f
#define CALIB_TOTAL_SAMPLE_NUM  (CALIB_DELAY_SAMPLE_NUM + CALIB_SAMPLE_NUM)

void imu_bias_calib_init(imu_bias_calib_t *state)
{
    if (state == NULL) {
        return;
    }
    memset(state, 0, sizeof(*state));
}

uint8_t imu_bias_calib_is_done(const imu_bias_calib_t *state)
{
    return (state != NULL) ? state->calib_phase : 0u;
}

float imu_bias_calib_progress(const imu_bias_calib_t *state)
{
    if (state == NULL) {
        return 0.0f;
    }
    if (state->calib_phase != 0u) {
        return 1.0f;
    }

    uint32_t done = state->calib_delay_cnt;
    if (done >= CALIB_DELAY_SAMPLE_NUM) {
        done = CALIB_DELAY_SAMPLE_NUM + (uint32_t)state->calib_sample_cnt;
    }
    return (float)done / (float)CALIB_TOTAL_SAMPLE_NUM;
}

uint8_t imu_bias_calib_stage(const imu_bias_calib_t *state)
{
    if (state == NULL) {
        return IMU_BIAS_STAGE_WARMUP;
    }
    if (state->calib_phase != 0u) {
        return IMU_BIAS_STAGE_DONE;
    }
    return (state->calib_delay_cnt < CALIB_DELAY_SAMPLE_NUM)
               ? IMU_BIAS_STAGE_WARMUP
               : IMU_BIAS_STAGE_CALIB;
}

void imu_bias_calib_update(imu_bias_calib_t *state,
                           const mpu9250_sample_t *input,
                           mpu9250_sample_t *output)
{
    if (state == NULL || input == NULL || output == NULL) {
        return;
    }

    if (state->calib_phase == 0) {
        if (state->calib_delay_cnt < CALIB_DELAY_SAMPLE_NUM) {
            /* ---------- 上电等待：不校准，偏置保持 0 ---------- */
            state->calib_delay_cnt++;
        } else {
            /* ---------- 静止零偏校准 ---------- */
            state->calib_acc_sum[0] += input->acc_x;
            state->calib_acc_sum[1] += input->acc_y;
            state->calib_acc_sum[2] += input->acc_z;
            state->calib_gyro_sum[0] += input->gyro_x;
            state->calib_gyro_sum[1] += input->gyro_y;
            state->calib_gyro_sum[2] += input->gyro_z;
            state->calib_sample_cnt++;

            if (state->calib_sample_cnt >= CALIB_SAMPLE_NUM) {
                const float inv_n = 1.0f / (float)CALIB_SAMPLE_NUM;

                /* 静止且 Z 轴朝上时，加速度 Z 扣除 1g 重力 */
                state->accel_bias[0] = state->calib_acc_sum[0] * inv_n;
                state->accel_bias[1] = state->calib_acc_sum[1] * inv_n;
                state->accel_bias[2] = state->calib_acc_sum[2] * inv_n - 1.0f;

                state->gyro_bias[0] = state->calib_gyro_sum[0] * inv_n;
                state->gyro_bias[1] = state->calib_gyro_sum[1] * inv_n;
                state->gyro_bias[2] = state->calib_gyro_sum[2] * inv_n;

                state->prev_input = *input;
                state->calib_phase = 1;
            }
        }
    } else {
        /* ---------- 陀螺仪零偏在线跟踪 ---------- */
        const float dx = input->gyro_x - state->prev_input.gyro_x;
        const float dy = input->gyro_y - state->prev_input.gyro_y;
        const float dz = input->gyro_z - state->prev_input.gyro_z;
        const float delta_sq = dx * dx + dy * dy + dz * dz;

        if (delta_sq < GYRO_TRACK_THRES_SQ) {
            state->gyro_bias[0] += TRACK_ALPHA * (input->gyro_x - state->gyro_bias[0]);
            state->gyro_bias[1] += TRACK_ALPHA * (input->gyro_y - state->gyro_bias[1]);
            state->gyro_bias[2] += TRACK_ALPHA * (input->gyro_z - state->gyro_bias[2]);
        }
    }

    state->prev_input = *input;

    output->temp_c = input->temp_c;
    output->acc_x = input->acc_x - state->accel_bias[0];
    output->acc_y = input->acc_y - state->accel_bias[1];
    output->acc_z = input->acc_z - state->accel_bias[2];
    output->gyro_x = input->gyro_x - state->gyro_bias[0];
    output->gyro_y = input->gyro_y - state->gyro_bias[1];
    output->gyro_z = input->gyro_z - state->gyro_bias[2];
    output->mag_x = input->mag_x;
    output->mag_y = input->mag_y;
    output->mag_z = input->mag_z;
}
