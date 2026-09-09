/*
 * IMU 偏置校准与静止零偏跟踪实现
 *
 * 参数按 100Hz 采样频率整定（对应 main.c 的 IMU_PERIOD_MS = 10ms）。
 */
#include "imu_bias_calib.h"

#include <math.h>

/* 校准采样数：100Hz 下约 2 秒 */
#define CALIB_SAMPLE_NUM        200
/* 静止判定阈值（帧间角速度变化，单位 °/s） */
#define GYRO_TRACK_THRES_DPS    1.0f
#define GYRO_TRACK_THRES_SQ     (GYRO_TRACK_THRES_DPS * GYRO_TRACK_THRES_DPS)
/* 陀螺零偏跟踪低通系数 */
#define TRACK_ALPHA             0.002f
/* 加速度零偏标定的水平度门限：|mean_z| >= 0.9 约等于倾角 < 25° */
#define ACCEL_LEVEL_MIN_Z       0.9f

void imu_bias_calib_init(imu_bias_calib_t *s)
{
    if (s == NULL) {
        return;
    }

    for (int i = 0; i < 3; i++) {
        s->gyro_bias[i] = 0.0f;
        s->accel_bias[i] = 0.0f;
        s->calib_acc_sum[i] = 0.0f;
        s->calib_gyro_sum[i] = 0.0f;
    }

    s->calib_sample_cnt = 0;
    s->calib_phase = 0;
    s->prev_input = (mpu9250_sample_t){0};
}

bool imu_bias_calib_is_done(const imu_bias_calib_t *s)
{
    return (s != NULL) && (s->calib_phase != 0);
}

void imu_bias_calib_update(imu_bias_calib_t *s,
                           const mpu9250_sample_t *input,
                           mpu9250_sample_t *output)
{
    if (s == NULL || input == NULL || output == NULL) {
        return;
    }

    if (s->calib_phase == 0) {
        /* ---------- 上电静止校准：累计均值求零偏 ---------- */
        s->calib_acc_sum[0] += input->acc_x;
        s->calib_acc_sum[1] += input->acc_y;
        s->calib_acc_sum[2] += input->acc_z;
        s->calib_gyro_sum[0] += input->gyro_x;
        s->calib_gyro_sum[1] += input->gyro_y;
        s->calib_gyro_sum[2] += input->gyro_z;
        s->calib_sample_cnt++;

        if (s->calib_sample_cnt >= CALIB_SAMPLE_NUM) {
            const float inv_n = 1.0f / (float)CALIB_SAMPLE_NUM;

            /* 陀螺仪零偏：静止时三轴角速度均值即为零偏 */
            s->gyro_bias[0] = s->calib_gyro_sum[0] * inv_n;
            s->gyro_bias[1] = s->calib_gyro_sum[1] * inv_n;
            s->gyro_bias[2] = s->calib_gyro_sum[2] * inv_n;

            /* 加速度计零偏：仅在接近水平静止时标定，Z 轴扣除 1g 重力 */
            float mean_x = s->calib_acc_sum[0] * inv_n;
            float mean_y = s->calib_acc_sum[1] * inv_n;
            float mean_z = s->calib_acc_sum[2] * inv_n;
            if (fabsf(mean_z) >= ACCEL_LEVEL_MIN_Z) {
                s->accel_bias[0] = mean_x;
                s->accel_bias[1] = mean_y;
                s->accel_bias[2] = mean_z - (mean_z >= 0.0f ? 1.0f : -1.0f);
            } else {
                s->accel_bias[0] = 0.0f;
                s->accel_bias[1] = 0.0f;
                s->accel_bias[2] = 0.0f;
            }

            s->prev_input = *input;
            s->calib_phase = 1;
        }
    } else {
        /* ---------- 运行期：静止时低通跟踪陀螺零偏 ---------- */
        float dx = input->gyro_x - s->prev_input.gyro_x;
        float dy = input->gyro_y - s->prev_input.gyro_y;
        float dz = input->gyro_z - s->prev_input.gyro_z;
        float delta_sq = dx * dx + dy * dy + dz * dz;

        if (delta_sq < GYRO_TRACK_THRES_SQ) {
            s->gyro_bias[0] += TRACK_ALPHA * (input->gyro_x - s->gyro_bias[0]);
            s->gyro_bias[1] += TRACK_ALPHA * (input->gyro_y - s->gyro_bias[1]);
            s->gyro_bias[2] += TRACK_ALPHA * (input->gyro_z - s->gyro_bias[2]);
        }

        s->prev_input = *input;
    }

    /* ---------- 应用偏置补偿（磁力计原样透传） ---------- */
    output->acc_x = input->acc_x - s->accel_bias[0];
    output->acc_y = input->acc_y - s->accel_bias[1];
    output->acc_z = input->acc_z - s->accel_bias[2];
    output->gyro_x = input->gyro_x - s->gyro_bias[0];
    output->gyro_y = input->gyro_y - s->gyro_bias[1];
    output->gyro_z = input->gyro_z - s->gyro_bias[2];
    output->mag_x = input->mag_x;
    output->mag_y = input->mag_y;
    output->mag_z = input->mag_z;
}
