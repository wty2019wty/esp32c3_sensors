/*
 * Mahony AHRS 姿态融合实现
 */
#include "mahony.h"

#include <math.h>
#include <stdbool.h>

#define MAHONY_RAD_TO_DEG   (57.29577951308232f)
#define MAHONY_DEG_TO_RAD   (0.017453292519943295f)

/* 快速平方根倒数（Newton 迭代） */
static float mahony_inv_sqrt(float x)
{
    return 1.0f / sqrtf(x);
}

void mahony_init(mahony_t *m, float kp, float ki)
{
    if (m == NULL) {
        return;
    }
    m->q0 = 1.0f;
    m->q1 = 0.0f;
    m->q2 = 0.0f;
    m->q3 = 0.0f;
    m->integral_fb_x = 0.0f;
    m->integral_fb_y = 0.0f;
    m->integral_fb_z = 0.0f;
    m->two_kp = 2.0f * kp;
    m->two_ki = 2.0f * ki;
}

void mahony_update(mahony_t *m,
                   float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float mx, float my, float mz,
                   float dt)
{
    if (m == NULL || dt <= 0.0f) {
        return;
    }

    /* 陀螺仪由 °/s 转 rad/s */
    gx *= MAHONY_DEG_TO_RAD;
    gy *= MAHONY_DEG_TO_RAD;
    gz *= MAHONY_DEG_TO_RAD;

    float recip_norm;
    float q0 = m->q0;
    float q1 = m->q1;
    float q2 = m->q2;
    float q3 = m->q3;

    float half_vx, half_vy, half_vz;
    float half_wx, half_wy, half_wz;
    float half_ex, half_ey, half_ez;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    bool acc_valid = !((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f));
    bool mag_valid = !((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f));

    if (acc_valid) {
        /* 归一化加速度计 */
        recip_norm = mahony_inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        /* 归一化磁力计（如有） */
        float hx = 0.0f, hy = 0.0f, bx = 0.0f, bz = 0.0f;
        if (mag_valid) {
            recip_norm = mahony_inv_sqrt(mx * mx + my * my + mz * mz);
            mx *= recip_norm;
            my *= recip_norm;
            mz *= recip_norm;
        }

        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        /* 地球磁场参考方向（仅磁力计有效时） */
        if (mag_valid) {
            hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
            hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
            bx = sqrtf(hx * hx + hy * hy);
            bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
        }

        /* 估计重力方向 */
        half_vx = q1q3 - q0q2;
        half_vy = q0q1 + q2q3;
        half_vz = q0q0 - 0.5f + q3q3;

        /* 误差：重力/磁场测量与估计的叉积 */
        half_ex = (ay * half_vz - az * half_vy);
        half_ey = (az * half_vx - ax * half_vz);
        half_ez = (ax * half_vy - ay * half_vx);

        if (mag_valid) {
            half_wx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
            half_wy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
            half_wz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

            half_ex += (my * half_wz - mz * half_wy);
            half_ey += (mz * half_wx - mx * half_wz);
            half_ez += (mx * half_wy - my * half_wx);
        }

        /* 积分反馈 */
        if (m->two_ki > 0.0f) {
            m->integral_fb_x += m->two_ki * half_ex * dt;
            m->integral_fb_y += m->two_ki * half_ey * dt;
            m->integral_fb_z += m->two_ki * half_ez * dt;
            gx += m->integral_fb_x;
            gy += m->integral_fb_y;
            gz += m->integral_fb_z;
        } else {
            m->integral_fb_x = 0.0f;
            m->integral_fb_y = 0.0f;
            m->integral_fb_z = 0.0f;
        }

        /* 比例反馈 */
        gx += m->two_kp * half_ex;
        gy += m->two_kp * half_ey;
        gz += m->two_kp * half_ez;
    }

    /* 四元数积分 */
    float half_dt = 0.5f * dt;
    float qa = q0;
    float qb = q1;
    float qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz) * half_dt;
    q1 += (qa * gx + qc * gz - q3 * gy) * half_dt;
    q2 += (qa * gy - qb * gz + q3 * gx) * half_dt;
    q3 += (qa * gz + qb * gy - qc * gx) * half_dt;

    /* 归一化四元数 */
    recip_norm = mahony_inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    m->q0 = q0 * recip_norm;
    m->q1 = q1 * recip_norm;
    m->q2 = q2 * recip_norm;
    m->q3 = q3 * recip_norm;
}

void mahony_get_euler(const mahony_t *m, float *roll, float *pitch, float *yaw)
{
    if (m == NULL || roll == NULL || pitch == NULL || yaw == NULL) {
        return;
    }

    float q0 = m->q0;
    float q1 = m->q1;
    float q2 = m->q2;
    float q3 = m->q3;

    *roll = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * MAHONY_RAD_TO_DEG;

    float sinp = -2.0f * (q1 * q3 - q0 * q2);
    if (sinp > 1.0f) {
        sinp = 1.0f;
    } else if (sinp < -1.0f) {
        sinp = -1.0f;
    }
    *pitch = asinf(sinp) * MAHONY_RAD_TO_DEG;

    *yaw = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * MAHONY_RAD_TO_DEG;
}
