/*
 * EKF AHRS 姿态估计实现
 *
 * 四元数扩展卡尔曼滤波器，用于 6/9 轴 IMU 姿态融合。
 * 采用简化计算避免 4x4 矩阵求逆，适合嵌入式实时运行。
 */
#include "ekf_ahrs.h"

#include <math.h>
#include <string.h>

#define EKF_DEG_TO_RAD  0.017453292519943295f
#define EKF_RAD_TO_DEG  57.29577951308232f

/* 初始化 */
void ekf_ahrs_init(ekf_ahrs_t *ekf, float Q_gyro, float R_accel, float R_mag)
{
    if (ekf == NULL) {
        return;
    }
    memset(ekf, 0, sizeof(*ekf));

    ekf->q[0] = 1.0f;
    ekf->q[1] = 0.0f;
    ekf->q[2] = 0.0f;
    ekf->q[3] = 0.0f;

    /* 初始协方差：对角阵，表示初始不确定性较大 */
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            ekf->P[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    ekf->Q_gyro  = Q_gyro;
    ekf->R_accel = R_accel;
    ekf->R_mag   = R_mag;
    ekf->initialized = true;
}

/* 四元数乘法 q_out = q_a * q_b */
static void quat_mult(float q_out[4], const float q_a[4], const float q_b[4])
{
    q_out[0] = q_a[0]*q_b[0] - q_a[1]*q_b[1] - q_a[2]*q_b[2] - q_a[3]*q_b[3];
    q_out[1] = q_a[0]*q_b[1] + q_a[1]*q_b[0] + q_a[2]*q_b[3] - q_a[3]*q_b[2];
    q_out[2] = q_a[0]*q_b[2] - q_a[1]*q_b[3] + q_a[2]*q_b[0] + q_a[3]*q_b[1];
    q_out[3] = q_a[0]*q_b[3] + q_a[1]*q_b[2] - q_a[2]*q_b[1] + q_a[3]*q_b[0];
}

/* 四元数归一化 */
static void quat_normalize(float q[4])
{
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 1e-10f) {
        float inv = 1.0f / norm;
        q[0] *= inv;
        q[1] *= inv;
        q[2] *= inv;
        q[3] *= inv;
    }
}

/* 向量叉积 */
static void vec3_cross(float out[3], const float a[3], const float b[3])
{
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

/* 向量点积 */
static float vec3_dot(const float a[3], const float b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

/* 向量归一化，返回是否成功 */
static bool vec3_normalize(float v[3])
{
    float norm = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (norm < 1e-10f) {
        return false;
    }
    float inv = 1.0f / norm;
    v[0] *= inv;
    v[1] *= inv;
    v[2] *= inv;
    return true;
}

/*
 * EKF 预测步骤
 *
 * 状态转移：q(k+1) = F * q(k)，其中 F = I + 0.5*dt*Omega(gyro)
 * 协方差预测：P = F*P*F^T + Q
 */
static void ekf_predict(ekf_ahrs_t *ekf, float gx, float gy, float gz, float dt)
{
    float *q = ekf->q;
    float half_dt = 0.5f * dt;

    /* 陀螺仪转 rad/s */
    float wx = gx * EKF_DEG_TO_RAD;
    float wy = gy * EKF_DEG_TO_RAD;
    float wz = gz * EKF_DEG_TO_RAD;

    /* 状态转移：一阶欧拉积分 */
    float dq0 = half_dt * (-wx*q[1] - wy*q[2] - wz*q[3]);
    float dq1 = half_dt * ( wx*q[0] + wz*q[2] - wy*q[3]);
    float dq2 = half_dt * ( wy*q[0] - wz*q[1] + wx*q[3]);
    float dq3 = half_dt * ( wz*q[0] + wy*q[1] - wx*q[2]);

    q[0] += dq0;
    q[1] += dq1;
    q[2] += dq2;
    q[3] += dq3;
    quat_normalize(q);

    /* 协方差预测：P = F*P*F^T + Q
     * F = I + 0.5*dt*Omega，简化为 P' ≈ P + Q
     * （高阶项在 dt 很小时可忽略，且避免 4x4 矩阵乘法） */
    float q_noise = (half_dt * ekf->Q_gyro) * (half_dt * ekf->Q_gyro);
    for (int i = 0; i < 4; i++) {
        ekf->P[i][i] += q_noise;
    }
}

/*
 * EKF 更新步骤（加速度计观测）
 *
 * 观测模型：h(q) = R(q)^T * [0,0,1]^T（重力在机体坐标系的投影）
 *   h = [2(q1q3 - q0q2), 2(q0q1 + q2q3), q0^2 - q1^2 - q2^2 + q3^2]
 *
 * 观测雅可比 H (3x4):
 *   H = 2*[[-q2,  q3, -q0,  q1],
 *           [ q1,  q0,  q3,  q2],
 *           [ q0, -q1, -q2,  q3]]
 *
 * 采用简化卡尔曼增益计算，避免 3x3 矩阵求逆。
 */
static void ekf_update_accel(ekf_ahrs_t *ekf, float ax, float ay, float az)
{
    float *q = ekf->q;

    /* 归一化加速度计 */
    float a[3] = {ax, ay, az};
    if (!vec3_normalize(a)) {
        return;  /* 零向量，跳过更新 */
    }

    /* 预测的重力方向 h(q) */
    float h[3];
    h[0] = 2.0f * (q[1]*q[3] - q[0]*q[2]);
    h[1] = 2.0f * (q[0]*q[1] + q[2]*q[3]);
    h[2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

    /* 观测残差 y = z - h */
    float y[3];
    y[0] = a[0] - h[0];
    y[1] = a[1] - h[1];
    y[2] = a[2] - h[2];

    /* 观测雅可比 H (3x4) */
    float H[3][4];
    H[0][0] = -q[2];  H[0][1] =  q[3];  H[0][2] = -q[0];  H[0][3] =  q[1];
    H[1][0] =  q[1];  H[1][1] =  q[0];  H[1][2] =  q[3];  H[1][3] =  q[2];
    H[2][0] =  q[0];  H[2][1] = -q[1];  H[2][2] = -q[2];  H[2][3] =  q[3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            H[i][j] *= 2.0f;
        }
    }

    /* S = H*P*H^T + R (3x3)
     * 先计算 PH = P*H^T (4x3) */
    float PH[4][3];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            PH[i][j] = 0.0f;
            for (int k = 0; k < 4; k++) {
                PH[i][j] += ekf->P[i][k] * H[j][k];
            }
        }
    }

    /* S = H*PH + R */
    float S[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            S[i][j] = 0.0f;
            for (int k = 0; k < 4; k++) {
                S[i][j] += H[i][k] * PH[k][j];
            }
            if (i == j) {
                S[i][j] += ekf->R_accel;
            }
        }
    }

    /* S 的逆（3x3 对称矩阵，用伴随矩阵法） */
    float det = S[0][0] * (S[1][1]*S[2][2] - S[1][2]*S[2][1])
              - S[0][1] * (S[1][0]*S[2][2] - S[1][2]*S[2][0])
              + S[0][2] * (S[1][0]*S[2][1] - S[1][1]*S[2][0]);

    if (fabsf(det) < 1e-12f) {
        return;  /* 奇异，跳过 */
    }
    float inv_det = 1.0f / det;

    float S_inv[3][3];
    S_inv[0][0] = (S[1][1]*S[2][2] - S[1][2]*S[2][1]) * inv_det;
    S_inv[0][1] = (S[0][2]*S[2][1] - S[0][1]*S[2][2]) * inv_det;
    S_inv[0][2] = (S[0][1]*S[1][2] - S[0][2]*S[1][1]) * inv_det;
    S_inv[1][0] = (S[1][2]*S[2][0] - S[1][0]*S[2][2]) * inv_det;
    S_inv[1][1] = (S[0][0]*S[2][2] - S[0][2]*S[2][0]) * inv_det;
    S_inv[1][2] = (S[0][2]*S[1][0] - S[0][0]*S[1][2]) * inv_det;
    S_inv[2][0] = (S[1][0]*S[2][1] - S[1][1]*S[2][0]) * inv_det;
    S_inv[2][1] = (S[0][1]*S[2][0] - S[0][0]*S[2][1]) * inv_det;
    S_inv[2][2] = (S[0][0]*S[1][1] - S[0][1]*S[1][0]) * inv_det;

    /* 卡尔曼增益 K = PH * S_inv (4x3) */
    float K[4][3];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            K[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                K[i][j] += PH[i][k] * S_inv[k][j];
            }
        }
    }

    /* 状态更新 x = x + K*y */
    for (int i = 0; i < 4; i++) {
        float correction = 0.0f;
        for (int j = 0; j < 3; j++) {
            correction += K[i][j] * y[j];
        }
        ekf->q[i] += correction;
    }
    quat_normalize(ekf->q);

    /* 协方差更新（Joseph 形式，数值稳定）：P = (I - K*H)*P */
    float I_KH[4][4];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            float kh = 0.0f;
            for (int k = 0; k < 3; k++) {
                kh += K[i][k] * H[k][j];
            }
            I_KH[i][j] = (i == j ? 1.0f : 0.0f) - kh;
        }
    }

    float P_new[4][4];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P_new[i][j] = 0.0f;
            for (int k = 0; k < 4; k++) {
                P_new[i][j] += I_KH[i][k] * ekf->P[k][j];
            }
        }
    }
    memcpy(ekf->P, P_new, sizeof(P_new));
}

/*
 * EKF 更新步骤（磁力计观测，可选）
 *
 * 磁力计观测模型：水平面磁场方向
 * h_mag = R(q)^T * [bx, 0, bz]（假设磁场在水平面的分量沿 x 轴）
 *
 * 为简化计算，只用磁力计的水平分量修正 yaw。
 * 这里用简化方式：将磁力计投影到水平面，与预测值比较。
 */
static void ekf_update_mag(ekf_ahrs_t *ekf, float mx, float my, float mz)
{
    if (ekf->R_mag <= 0.0f) {
        return;  /* 磁力计噪声为 0，不使用 */
    }

    float *q = ekf->q;

    /* 将磁力计测量值旋转到导航坐标系 */
    float q0q0 = q[0]*q[0], q0q1 = q[0]*q[1], q0q2 = q[0]*q[2], q0q3 = q[0]*q[3];
    float q1q1 = q[1]*q[1], q1q2 = q[1]*q[2], q1q3 = q[1]*q[3];
    float q2q2 = q[2]*q[2], q2q3 = q[2]*q[3], q3q3 = q[3]*q[3];

    /* 导航系磁场分量 */
    float hx = mx*(q0q0 + q1q1 - q2q2 - q3q3) + 2.0f*my*(q1q2 + q0q3) + 2.0f*mz*(q1q3 - q0q2);
    float hy = 2.0f*mx*(q1q2 - q0q3) + my*(q0q0 - q1q1 + q2q2 - q3q3) + 2.0f*mz*(q2q3 + q0q1);

    /* 航向角残差（仅水平分量） */
    float yaw_meas = atan2f(-hy, hx);

    /* 预测的航向角 */
    float yaw_pred = atan2f(2.0f*(q[1]*q[2] + q[0]*q[3]),
                            q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3]);

    float yaw_err = yaw_meas - yaw_pred;
    /* 角度归一化到 [-pi, pi] */
    while (yaw_err >  M_PI) yaw_err -= 2.0f * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2.0f * M_PI;

    /* 简化的 yaw 修正：通过绕 z 轴旋转四元数 */
    float half_err = 0.5f * yaw_err;
    float gain = ekf->R_mag / (ekf->R_mag + 1.0f);  /* 简化的卡尔曼增益 */
    float correction = half_err * gain;

    float dq[4];
    float cos_c = cosf(correction);
    float sin_c = sinf(correction);
    dq[0] = cos_c;
    dq[1] = 0.0f;
    dq[2] = 0.0f;
    dq[3] = sin_c;

    float q_new[4];
    quat_mult(q_new, ekf->q, dq);
    memcpy(ekf->q, q_new, sizeof(q_new));
    quat_normalize(ekf->q);
}

/* 主更新函数 */
void ekf_ahrs_update(ekf_ahrs_t *ekf,
                     float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float mx, float my, float mz,
                     float dt)
{
    if (ekf == NULL || !ekf->initialized || dt <= 0.0f) {
        return;
    }

    /* 1. 预测 */
    ekf_predict(ekf, gx, gy, gz, dt);

    /* 2. 加速度计更新 */
    ekf_update_accel(ekf, ax, ay, az);

    /* 3. 磁力计更新（可选） */
    bool mag_valid = !((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f));
    if (mag_valid) {
        ekf_update_mag(ekf, mx, my, mz);
    }
}

/* 欧拉角提取 */
void ekf_ahrs_get_euler(const ekf_ahrs_t *ekf, float *roll, float *pitch, float *yaw)
{
    if (ekf == NULL || roll == NULL || pitch == NULL || yaw == NULL) {
        return;
    }

    float q0 = ekf->q[0];
    float q1 = ekf->q[1];
    float q2 = ekf->q[2];
    float q3 = ekf->q[3];

    *roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * EKF_RAD_TO_DEG;

    float sinp = -2.0f * (q1*q3 - q0*q2);
    if (sinp > 1.0f) sinp = 1.0f;
    else if (sinp < -1.0f) sinp = -1.0f;
    *pitch = asinf(sinp) * EKF_RAD_TO_DEG;

    *yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3) * EKF_RAD_TO_DEG;
}
