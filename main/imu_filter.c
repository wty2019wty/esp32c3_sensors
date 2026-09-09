/*
 * IMU 二阶低通滤波器实现（LP2 biquad，Butterworth 型）
 *
 * 设计公式与系数布局移植自 stm32f103 工程的 IMU_FilterPortable.h。
 */
#include "imu_filter.h"

#include <math.h>

#define IMU_FILTER_PI  3.14159265358979323846f

/* 复位单个滤波器状态 */
static void imu_filter_biquad_reset(imu_filter_biquad_t *s)
{
    s->x1 = 0.0f;
    s->x2 = 0.0f;
    s->y1 = 0.0f;
    s->y2 = 0.0f;
    s->warmup_count = 0;
}

/*
 * 设计二阶低通滤波器系数（双线性变换 + Butterworth Q=1/sqrt(2)）
 * 采样率或截止频率非法时退化为直通（b0=1，其余为 0）。
 */
static void imu_filter_design_lp2(float sample_hz,
                                  float cutoff_hz,
                                  imu_filter_coeff_t *coeff)
{
    if (cutoff_hz <= 0.0f || sample_hz <= 0.0f || cutoff_hz >= sample_hz * 0.5f) {
        coeff->b0 = 1.0f;
        coeff->b1 = 0.0f;
        coeff->b2 = 0.0f;
        coeff->a1 = 0.0f;
        coeff->a2 = 0.0f;
        return;
    }

    const float fr = sample_hz / cutoff_hz;
    const float ohm = tanf(IMU_FILTER_PI / fr);
    const float cos_term = cosf(IMU_FILTER_PI / 4.0f);
    const float c = 1.0f + 2.0f * cos_term * ohm + ohm * ohm;
    const float b0 = (ohm * ohm) / c;

    coeff->b0 = b0;
    coeff->b1 = 2.0f * b0;
    coeff->b2 = b0;
    coeff->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
    coeff->a2 = (1.0f - 2.0f * cos_term * ohm + ohm * ohm) / c;
}

/* 单轴处理：预热期内直通，之后执行差分方程 */
static float imu_filter_biquad_process(float input,
                                       imu_filter_biquad_t *s,
                                       const imu_filter_coeff_t *coeff)
{
    float y;

    if (s->warmup_count < s->warmup_limit) {
        y = input;
        s->warmup_count++;
    } else {
        y = coeff->b0 * input
          + coeff->b1 * s->x1
          + coeff->b2 * s->x2
          - coeff->a1 * s->y1
          - coeff->a2 * s->y2;
    }

    s->x2 = s->x1;
    s->x1 = input;
    s->y2 = s->y1;
    s->y1 = y;

    return y;
}

void imu_filter_init(imu_filter_t *f,
                     float sample_hz,
                     float accel_cutoff_hz,
                     float gyro_cutoff_hz,
                     uint16_t warmup_limit)
{
    if (f == NULL) {
        return;
    }

    for (int i = 0; i < 3; i++) {
        imu_filter_biquad_reset(&f->accel_state[i]);
        f->accel_state[i].warmup_limit = warmup_limit;
        imu_filter_design_lp2(sample_hz, accel_cutoff_hz, &f->accel_coeff[i]);

        imu_filter_biquad_reset(&f->gyro_state[i]);
        f->gyro_state[i].warmup_limit = warmup_limit;
        imu_filter_design_lp2(sample_hz, gyro_cutoff_hz, &f->gyro_coeff[i]);
    }
}

void imu_filter_process(const mpu9250_sample_t *in,
                        mpu9250_sample_t *out,
                        imu_filter_t *f)
{
    if (in == NULL || out == NULL || f == NULL) {
        return;
    }

    out->acc_x = imu_filter_biquad_process(in->acc_x, &f->accel_state[0], &f->accel_coeff[0]);
    out->acc_y = imu_filter_biquad_process(in->acc_y, &f->accel_state[1], &f->accel_coeff[1]);
    out->acc_z = imu_filter_biquad_process(in->acc_z, &f->accel_state[2], &f->accel_coeff[2]);

    out->gyro_x = imu_filter_biquad_process(in->gyro_x, &f->gyro_state[0], &f->gyro_coeff[0]);
    out->gyro_y = imu_filter_biquad_process(in->gyro_y, &f->gyro_state[1], &f->gyro_coeff[1]);
    out->gyro_z = imu_filter_biquad_process(in->gyro_z, &f->gyro_state[2], &f->gyro_coeff[2]);

    /* 磁力计不滤波，原样透传 */
    out->mag_x = in->mag_x;
    out->mag_y = in->mag_y;
    out->mag_z = in->mag_z;
}
