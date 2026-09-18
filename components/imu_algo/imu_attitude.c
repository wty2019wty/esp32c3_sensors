#include "imu_attitude.h"

#include <math.h>

#define ATT6_DEG2RAD 0.01745329251994329577f
#define ATT6_RAD2DEG 57.295779513082320876f

static float att6_inv_sqrt(float x)
{
    if (x <= 0.0f) {
        return 0.0f;
    }
    return 1.0f / sqrtf(x);
}

static float att6_clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

void imu_attitude_reset(imu_attitude_t *state)
{
    if (state == NULL) {
        return;
    }
    state->q0 = 1.0f;
    state->q1 = 0.0f;
    state->q2 = 0.0f;
    state->q3 = 0.0f;
    state->ex_int = 0.0f;
    state->ey_int = 0.0f;
    state->ez_int = 0.0f;
    state->roll_deg = 0.0f;
    state->pitch_deg = 0.0f;
    state->yaw_deg = 0.0f;
}

void imu_attitude_init(imu_attitude_t *state, float kp, float ki)
{
    if (state == NULL) {
        return;
    }
    imu_attitude_reset(state);
    state->kp = kp;
    state->ki = ki;
}

void imu_attitude_update(imu_attitude_t *state,
                         const mpu9250_sample_t *data,
                         float dt_sec)
{
    if (state == NULL || data == NULL || dt_sec <= 0.0f) {
        return;
    }

    float q0 = state->q0;
    float q1 = state->q1;
    float q2 = state->q2;
    float q3 = state->q3;

    float gx_rad = data->gyro_x * ATT6_DEG2RAD;
    float gy_rad = data->gyro_y * ATT6_DEG2RAD;
    float gz_rad = data->gyro_z * ATT6_DEG2RAD;

    const float norm = att6_inv_sqrt(data->acc_x * data->acc_x +
                                     data->acc_y * data->acc_y +
                                     data->acc_z * data->acc_z);
    if (norm > 0.0f) {
        const float ax_n = data->acc_x * norm;
        const float ay_n = data->acc_y * norm;
        const float az_n = data->acc_z * norm;

        const float vx = 2.0f * (q1 * q3 - q0 * q2);
        const float vy = 2.0f * (q0 * q1 + q2 * q3);
        const float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        const float ex = ay_n * vz - az_n * vy;
        const float ey = az_n * vx - ax_n * vz;
        const float ez = ax_n * vy - ay_n * vx;

        state->ex_int += state->ki * ex * dt_sec;
        state->ey_int += state->ki * ey * dt_sec;
        state->ez_int += state->ki * ez * dt_sec;

        gx_rad += state->kp * ex + state->ex_int;
        gy_rad += state->kp * ey + state->ey_int;
        gz_rad += state->kp * ez + state->ez_int;
    }

    const float half_t = 0.5f * dt_sec;
    const float q0_last = q0;
    const float q1_last = q1;
    const float q2_last = q2;
    const float q3_last = q3;

    q0 += (-q1_last * gx_rad - q2_last * gy_rad - q3_last * gz_rad) * half_t;
    q1 += (q0_last * gx_rad + q2_last * gz_rad - q3_last * gy_rad) * half_t;
    q2 += (q0_last * gy_rad - q1_last * gz_rad + q3_last * gx_rad) * half_t;
    q3 += (q0_last * gz_rad + q1_last * gy_rad - q2_last * gx_rad) * half_t;

    const float qnorm = att6_inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (qnorm > 0.0f) {
        q0 *= qnorm;
        q1 *= qnorm;
        q2 *= qnorm;
        q3 *= qnorm;
    }

    state->q0 = q0;
    state->q1 = q1;
    state->q2 = q2;
    state->q3 = q3;

    /* 标准 ZYX：roll 绕 X，pitch 绕 Y，yaw 绕 Z */
    state->roll_deg = atan2f(2.0f * (q0 * q1 + q2 * q3),
                             1.0f - 2.0f * (q1 * q1 + q2 * q2)) * ATT6_RAD2DEG;

    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    sinp = att6_clamp(sinp, -1.0f, 1.0f);
    state->pitch_deg = asinf(sinp) * ATT6_RAD2DEG;

    state->yaw_deg = atan2f(2.0f * (q0 * q3 + q1 * q2),
                            1.0f - 2.0f * (q2 * q2 + q3 * q3)) * ATT6_RAD2DEG;
}
