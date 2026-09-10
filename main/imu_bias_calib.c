/*
 * IMU 偏置校准与静止零偏跟踪实现
 *
 * 参数按 100Hz 采样频率整定（对应 main.c 的 IMU_PERIOD_MS = 10ms）。
 */
#include "imu_bias_calib.h"

#include <math.h>

/* 校准采样数：100Hz 下约 2 秒（仅统计静止帧） */
#define CALIB_SAMPLE_NUM        200

/*
 * 上电校准期静止判据（尚无可靠零偏，用“绝对角速度 + 帧间变化”双重门槛）：
 *   - 绝对值门槛放宽到 2.5°/s，允许零偏本身接近 ±1~2°/s 的传感器；
 *   - 帧间变化门槛 1.0°/s，排除手持抖动与缓慢旋转。
 */
#define CALIB_ABS_THRES_DPS     2.5f
#define CALIB_DELTA_THRES_DPS   1.0f

/*
 * 运行期静止判据（零偏已知，看补偿后残差）：
 *   - 三轴残差绝对值均 < 0.6°/s；
 *   - 残差帧间变化 < 0.4°/s；
 *   - 连续 STILL_HOLD_FRAMES 帧（100Hz 下 300ms）才置位 still。
 */
#define STILL_RESID_THRES_DPS   0.6f
#define STILL_DELTA_THRES_DPS   0.4f
#define STILL_HOLD_FRAMES       30

/* 陀螺零偏跟踪低通系数：长时间静止后加快跟踪温漂 */
#define TRACK_ALPHA_SLOW        0.002f
#define TRACK_ALPHA_FAST        0.008f
#define TRACK_FAST_AFTER_FRAMES 300     /* 静止 3s 后切换到快跟踪 */

/* 加速度零偏标定的水平度门限：|mean_z| >= 0.9 约等于倾角 < 25° */
#define ACCEL_LEVEL_MIN_Z       0.9f

static float max_abs3(float a, float b, float c)
{
    float m = fabsf(a);
    float t = fabsf(b);
    if (t > m) {
        m = t;
    }
    t = fabsf(c);
    if (t > m) {
        m = t;
    }
    return m;
}

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
    s->still_cnt = 0;
    s->still = false;
}

bool imu_bias_calib_is_done(const imu_bias_calib_t *s)
{
    return (s != NULL) && (s->calib_phase != 0);
}

bool imu_bias_calib_is_still(const imu_bias_calib_t *s)
{
    return (s != NULL) && s->still;
}

void imu_bias_calib_update(imu_bias_calib_t *s,
                           const mpu9250_sample_t *input,
                           mpu9250_sample_t *output)
{
    if (s == NULL || input == NULL || output == NULL) {
        return;
    }

    if (s->calib_phase == 0) {
        /* ---------- 上电静止校准：仅累计“静止”样本求均值 ---------- */
        float dx = input->gyro_x - s->prev_input.gyro_x;
        float dy = input->gyro_y - s->prev_input.gyro_y;
        float dz = input->gyro_z - s->prev_input.gyro_z;
        float delta_sq = dx * dx + dy * dy + dz * dz;
        float abs_max = max_abs3(input->gyro_x, input->gyro_y, input->gyro_z);
        const float delta_thres_sq = CALIB_DELTA_THRES_DPS * CALIB_DELTA_THRES_DPS;
        bool quiet = (abs_max < CALIB_ABS_THRES_DPS) && (delta_sq < delta_thres_sq);

        if (!quiet) {
            /* 运动：清空累计，重新等待静止（prev 始终刷新，避免“冻结”假静止） */
            s->calib_acc_sum[0] = 0.0f;
            s->calib_acc_sum[1] = 0.0f;
            s->calib_acc_sum[2] = 0.0f;
            s->calib_gyro_sum[0] = 0.0f;
            s->calib_gyro_sum[1] = 0.0f;
            s->calib_gyro_sum[2] = 0.0f;
            s->calib_sample_cnt = 0;
        } else {
            s->calib_acc_sum[0] += input->acc_x;
            s->calib_acc_sum[1] += input->acc_y;
            s->calib_acc_sum[2] += input->acc_z;
            s->calib_gyro_sum[0] += input->gyro_x;
            s->calib_gyro_sum[1] += input->gyro_y;
            s->calib_gyro_sum[2] += input->gyro_z;
            s->calib_sample_cnt++;
        }

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
            s->still_cnt = 0;
            s->still = false;
            s->calib_phase = 1;
        } else {
            s->prev_input = *input;
        }
    } else {
        /* ---------- 运行期：基于补偿后残差的静止检测 + 自适应零偏跟踪 ---------- */
        float rx = input->gyro_x - s->gyro_bias[0];
        float ry = input->gyro_y - s->gyro_bias[1];
        float rz = input->gyro_z - s->gyro_bias[2];

        float dx = input->gyro_x - s->prev_input.gyro_x;
        float dy = input->gyro_y - s->prev_input.gyro_y;
        float dz = input->gyro_z - s->prev_input.gyro_z;
        float delta_sq = dx * dx + dy * dy + dz * dz;
        float resid_max = max_abs3(rx, ry, rz);
        const float still_delta_sq = STILL_DELTA_THRES_DPS * STILL_DELTA_THRES_DPS;

        if (resid_max < STILL_RESID_THRES_DPS && delta_sq < still_delta_sq) {
            if (s->still_cnt < 0xFFFFu) {
                s->still_cnt++;
            }
        } else {
            s->still_cnt = 0;
            s->still = false;
        }

        /* 连续静止达到保持帧数后置位 */
        s->still = (s->still_cnt >= STILL_HOLD_FRAMES);

        if (s->still) {
            float alpha = (s->still_cnt >= TRACK_FAST_AFTER_FRAMES)
                              ? TRACK_ALPHA_FAST
                              : TRACK_ALPHA_SLOW;
            s->gyro_bias[0] += alpha * (input->gyro_x - s->gyro_bias[0]);
            s->gyro_bias[1] += alpha * (input->gyro_y - s->gyro_bias[1]);
            s->gyro_bias[2] += alpha * (input->gyro_z - s->gyro_bias[2]);
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
