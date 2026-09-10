/*
 * IMU 偏置校准与静止零偏跟踪实现
 *
 * 参数按 100Hz 采样频率整定（对应 main.c 的 IMU_PERIOD_MS = 10ms）。
 */
#include "imu_bias_calib.h"

#include <math.h>

/* 校准采样数：100Hz 下约 2 秒（仅统计静止帧） */
#define CALIB_SAMPLE_NUM        200
/* 校准超时：100Hz 下 10s。持续运动/零偏过大时强制结束，避免姿态永不输出 */
#define CALIB_TIMEOUT_FRAMES    1000

/*
 * 上电校准期静止判据（尚无可靠零偏，用“绝对角速度 + 帧间变化 + 加速度量级”）：
 *   - 绝对值门槛放宽到 2.5°/s，允许零偏本身接近 ±1~2°/s 的传感器；
 *   - 帧间变化门槛 1.0°/s，排除手持抖动与缓慢旋转；
 *   - 加速度模长须接近 1g，排除运动/失重时把线加速度当静止。
 */
#define CALIB_ABS_THRES_DPS     2.5f
#define CALIB_DELTA_THRES_DPS   1.0f
#define CALIB_ACC_NORM_MIN      0.85f
#define CALIB_ACC_NORM_MAX      1.15f

/*
 * 运行期静止判据（零偏已知，看补偿后残差 + 加速度）：
 *   - 三轴残差绝对值均 < 0.5°/s；
 *   - 残差帧间变化 < 0.35°/s；
 *   - 加速度模长在 [0.90, 1.10]g 且帧间变化小（排除推/拉/走动）；
 *   - 连续 STILL_HOLD_FRAMES 帧（100Hz 下 300ms）才置位 still。
 */
#define STILL_RESID_THRES_DPS   0.5f
#define STILL_DELTA_THRES_DPS   0.35f
#define STILL_ACC_NORM_MIN      0.90f
#define STILL_ACC_NORM_MAX      1.10f
#define STILL_ACC_DELTA_G       0.04f
#define STILL_HOLD_FRAMES       30
/*
 * 慢速兜底跟踪的可恢复残差上限：残差超过 STILL_RESID_THRES_DPS 时，
 * 只要仍在此范围内且帧间稳定，就用慢速低通把零偏拉回，避免温漂把残差
 * 推出阈值后永久锁死（残差正是跟踪要修正的量）。
 */
#define STILL_RECOVER_THRES_DPS 2.5f

/* 陀螺零偏跟踪低通系数：长时间静止后加快跟踪温漂 */
#define TRACK_ALPHA_SLOW        0.002f
#define TRACK_ALPHA_FAST        0.008f
#define TRACK_FAST_AFTER_FRAMES 300     /* 静止 3s 后切换到快跟踪 */

/* 加速度零偏标定的水平度门限：|mean_z| >= 0.9 约等于倾角 < 25° */
#define ACCEL_LEVEL_MIN_Z       0.9f

/* 温漂系数估计：静止跟踪时若温度相对上次明显变化，则用 Δbias/ΔT 更新 */
#define TEMPCO_MIN_DT_C         0.3f    /* 温度变化太小则不更新，避免噪声放大 */
#define TEMPCO_ALPHA            0.05f   /* 温漂系数一阶低通 */
#define TEMPCO_LIMIT_DPS_PER_C  0.15f   /* 限幅，防止异常样本污染 */
#define TEMPCO_CONF_THRES       0.3f    /* 置信度达到该值后才启用温漂外推 */

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

static float clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
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
        s->gyro_tempco[i] = 0.0f;
        s->last_track_bias[i] = 0.0f;
    }

    s->calib_sample_cnt = 0;
    s->calib_total_cnt = 0;
    s->calib_phase = 0;
    s->prev_input = (mpu9250_sample_t){0};
    s->still_cnt = 0;
    s->still = false;
    s->ref_temp_c = 0.0f;
    s->tempco_valid = 0.0f;
    s->last_track_temp_c = 0.0f;
    s->tempco_ready = false;
}

bool imu_bias_calib_is_done(const imu_bias_calib_t *s)
{
    return (s != NULL) && (s->calib_phase != 0);
}

bool imu_bias_calib_is_still(const imu_bias_calib_t *s)
{
    return (s != NULL) && s->still;
}

/* 当前应使用的陀螺零偏 = 参考温度处零偏 + 温漂外推 */
static void gyro_bias_applied(const imu_bias_calib_t *s,
                              float temp_c,
                              float bias_out[3])
{
    float dT = temp_c - s->ref_temp_c;
    if (!s->tempco_ready || s->tempco_valid < TEMPCO_CONF_THRES) {
        dT = 0.0f;
    }
    for (int i = 0; i < 3; i++) {
        bias_out[i] = s->gyro_bias[i] + s->gyro_tempco[i] * dT;
    }
}

void imu_bias_calib_update(imu_bias_calib_t *s,
                           const mpu9250_sample_t *input,
                           mpu9250_sample_t *output)
{
    if (s == NULL || input == NULL || output == NULL) {
        return;
    }

    const float acc_norm = sqrtf(input->acc_x * input->acc_x +
                                 input->acc_y * input->acc_y +
                                 input->acc_z * input->acc_z);
    const float dax = input->acc_x - s->prev_input.acc_x;
    const float day = input->acc_y - s->prev_input.acc_y;
    const float daz = input->acc_z - s->prev_input.acc_z;
    const float dacc_sq = dax * dax + day * day + daz * daz;

    if (s->calib_phase == 0) {
        /* ---------- 上电静止校准：仅累计“静止”样本求均值 ---------- */
        if (s->calib_total_cnt < 0xFFFFu) {
            s->calib_total_cnt++;
        }

        float dx = input->gyro_x - s->prev_input.gyro_x;
        float dy = input->gyro_y - s->prev_input.gyro_y;
        float dz = input->gyro_z - s->prev_input.gyro_z;
        float delta_sq = dx * dx + dy * dy + dz * dz;
        float abs_max = max_abs3(input->gyro_x, input->gyro_y, input->gyro_z);
        const float delta_thres_sq = CALIB_DELTA_THRES_DPS * CALIB_DELTA_THRES_DPS;
        bool quiet = (abs_max < CALIB_ABS_THRES_DPS) &&
                     (delta_sq < delta_thres_sq) &&
                     (acc_norm >= CALIB_ACC_NORM_MIN) &&
                     (acc_norm <= CALIB_ACC_NORM_MAX);

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

        /* 累计够静止样本，或等待超时（持续运动/零偏过大）时结束校准 */
        bool calib_finish = (s->calib_sample_cnt >= CALIB_SAMPLE_NUM) ||
                            (s->calib_total_cnt >= CALIB_TIMEOUT_FRAMES);

        if (calib_finish) {
            if (s->calib_sample_cnt > 0) {
                const float inv_n = 1.0f / (float)s->calib_sample_cnt;

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
            } else {
                /* 超时且从未捕获静止样本：零偏保持 0，
                   由运行期慢速兜底跟踪逐步收敛，至少保证融合能启动 */
                s->gyro_bias[0] = 0.0f;
                s->gyro_bias[1] = 0.0f;
                s->gyro_bias[2] = 0.0f;
                s->accel_bias[0] = 0.0f;
                s->accel_bias[1] = 0.0f;
                s->accel_bias[2] = 0.0f;
            }

            s->ref_temp_c = input->temp_c;
            s->last_track_temp_c = input->temp_c;
            s->last_track_bias[0] = s->gyro_bias[0];
            s->last_track_bias[1] = s->gyro_bias[1];
            s->last_track_bias[2] = s->gyro_bias[2];
            s->prev_input = *input;
            s->still_cnt = 0;
            s->still = false;
            s->calib_phase = 1;
        } else {
            s->prev_input = *input;
        }
    } else {
        /* ---------- 运行期：基于补偿后残差的静止检测 + 自适应零偏跟踪 ---------- */
        float bias_used[3];
        gyro_bias_applied(s, input->temp_c, bias_used);

        float rx = input->gyro_x - bias_used[0];
        float ry = input->gyro_y - bias_used[1];
        float rz = input->gyro_z - bias_used[2];

        float dx = input->gyro_x - s->prev_input.gyro_x;
        float dy = input->gyro_y - s->prev_input.gyro_y;
        float dz = input->gyro_z - s->prev_input.gyro_z;
        float delta_sq = dx * dx + dy * dy + dz * dz;
        float resid_max = max_abs3(rx, ry, rz);
        const float still_delta_sq = STILL_DELTA_THRES_DPS * STILL_DELTA_THRES_DPS;
        const float still_dacc_sq = STILL_ACC_DELTA_G * STILL_ACC_DELTA_G;

        /* 帧间稳定 + 加速度接近重力且稳定，才算“准静止”候选 */
        bool delta_quiet = (delta_sq < still_delta_sq);
        bool acc_quiet = (acc_norm >= STILL_ACC_NORM_MIN) &&
                         (acc_norm <= STILL_ACC_NORM_MAX) &&
                         (dacc_sq < still_dacc_sq);
        bool resid_ok = (resid_max < STILL_RESID_THRES_DPS);

        if (delta_quiet && acc_quiet) {
            if (s->still_cnt < 0xFFFFu) {
                s->still_cnt++;
            }
        } else {
            s->still_cnt = 0;
        }

        /* 置位静止：帧间稳定持续 + 残差足够小（供 ZUPT / 高 Ki 使用） */
        s->still = (s->still_cnt >= STILL_HOLD_FRAMES) && resid_ok;

        /*
         * 零偏跟踪：
         *   - 残差小且已置位静止时按慢/快双速跟踪；
         *   - 残差超阈值但仍在可恢复范围内时，仅用慢速兜底跟踪。
         * 跟踪的是“参考温度处的零偏”，同时用实际跟踪到的 Δbias/ΔT
         * 在线更新温漂系数。
         */
        if (delta_quiet && acc_quiet && resid_max < STILL_RECOVER_THRES_DPS) {
            float alpha = s->still
                              ? ((s->still_cnt >= TRACK_FAST_AFTER_FRAMES)
                                     ? TRACK_ALPHA_FAST
                                     : TRACK_ALPHA_SLOW)
                              : TRACK_ALPHA_SLOW;

            /* 目标：把参考温度处零偏修正，使补偿后残差→0。
             * 当前应用值 = bias_ref + tempco*(T-ref)，
             * 希望 bias_ref' + tempco*(T-ref) = input.gyro，
             * 即 bias_ref' = input.gyro - tempco*(T-ref)。 */
            float dT = input->temp_c - s->ref_temp_c;
            float target_ref[3];
            if (s->tempco_ready && s->tempco_valid >= TEMPCO_CONF_THRES) {
                target_ref[0] = input->gyro_x - s->gyro_tempco[0] * dT;
                target_ref[1] = input->gyro_y - s->gyro_tempco[1] * dT;
                target_ref[2] = input->gyro_z - s->gyro_tempco[2] * dT;
            } else {
                target_ref[0] = input->gyro_x;
                target_ref[1] = input->gyro_y;
                target_ref[2] = input->gyro_z;
            }

            s->gyro_bias[0] += alpha * (target_ref[0] - s->gyro_bias[0]);
            s->gyro_bias[1] += alpha * (target_ref[1] - s->gyro_bias[1]);
            s->gyro_bias[2] += alpha * (target_ref[2] - s->gyro_bias[2]);

            /* 温漂系数：两次跟踪之间若温度变化足够大，用 Δbias/ΔT 估计 */
            float d_track_t = input->temp_c - s->last_track_temp_c;
            if (fabsf(d_track_t) >= TEMPCO_MIN_DT_C) {
                for (int i = 0; i < 3; i++) {
                    float db = s->gyro_bias[i] - s->last_track_bias[i];
                    float est = clampf(db / d_track_t,
                                       -TEMPCO_LIMIT_DPS_PER_C,
                                       TEMPCO_LIMIT_DPS_PER_C);
                    s->gyro_tempco[i] += TEMPCO_ALPHA * (est - s->gyro_tempco[i]);
                }
                s->tempco_valid = s->tempco_valid + (1.0f - s->tempco_valid) * 0.2f;
                if (s->tempco_valid >= TEMPCO_CONF_THRES) {
                    s->tempco_ready = true;
                }
                s->last_track_temp_c = input->temp_c;
                s->last_track_bias[0] = s->gyro_bias[0];
                s->last_track_bias[1] = s->gyro_bias[1];
                s->last_track_bias[2] = s->gyro_bias[2];
            } else {
                /* 温度几乎不变时，持续刷新跟踪锚点，避免 Δbias 被时间拉开 */
                s->last_track_temp_c += 0.01f * (input->temp_c - s->last_track_temp_c);
                s->last_track_bias[0] += 0.01f * (s->gyro_bias[0] - s->last_track_bias[0]);
                s->last_track_bias[1] += 0.01f * (s->gyro_bias[1] - s->last_track_bias[1]);
                s->last_track_bias[2] += 0.01f * (s->gyro_bias[2] - s->last_track_bias[2]);
            }
        }

        s->prev_input = *input;
    }

    /* ---------- 应用偏置补偿（磁力计、温度原样透传） ---------- */
    float bias_out[3];
    if (s->calib_phase == 0) {
        /* 校准未完成时零偏尚未定，直接透传原始值，便于观察静止过程 */
        bias_out[0] = s->gyro_bias[0];
        bias_out[1] = s->gyro_bias[1];
        bias_out[2] = s->gyro_bias[2];
    } else {
        gyro_bias_applied(s, input->temp_c, bias_out);
    }

    output->acc_x = input->acc_x - s->accel_bias[0];
    output->acc_y = input->acc_y - s->accel_bias[1];
    output->acc_z = input->acc_z - s->accel_bias[2];
    output->gyro_x = input->gyro_x - bias_out[0];
    output->gyro_y = input->gyro_y - bias_out[1];
    output->gyro_z = input->gyro_z - bias_out[2];
    output->mag_x = input->mag_x;
    output->mag_y = input->mag_y;
    output->mag_z = input->mag_z;
    output->temp_c = input->temp_c;
}
