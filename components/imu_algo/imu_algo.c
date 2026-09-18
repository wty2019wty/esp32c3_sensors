#include "imu_algo.h"

#include <string.h>

void imu_pipeline_init(imu_pipeline_t *pipeline,
                       float sample_hz,
                       float acc_cutoff_hz,
                       float gyro_cutoff_hz,
                       uint16_t warmup_frames,
                       float att_kp,
                       float att_ki)
{
    if (pipeline == NULL) {
        return;
    }

    memset(pipeline, 0, sizeof(*pipeline));
    if (sample_hz <= 0.0f) {
        sample_hz = IMU_ALGO_DEFAULT_SAMPLE_HZ;
    }

    imu_filter_init(&pipeline->filter,
                    sample_hz,
                    acc_cutoff_hz,
                    gyro_cutoff_hz,
                    warmup_frames);
    imu_bias_calib_init(&pipeline->bias);
    imu_attitude_init(&pipeline->attitude, att_kp, att_ki);

    pipeline->sample_dt_sec = 1.0f / sample_hz;
    pipeline->ready = true;
}

void imu_pipeline_init_defaults(imu_pipeline_t *pipeline, float sample_hz)
{
    imu_pipeline_init(pipeline,
                      sample_hz,
                      IMU_ALGO_DEFAULT_ACC_CUTOFF_HZ,
                      IMU_ALGO_DEFAULT_GYRO_CUTOFF_HZ,
                      IMU_ALGO_DEFAULT_WARMUP_FRAMES,
                      IMU_ALGO_DEFAULT_ATT_KP,
                      IMU_ALGO_DEFAULT_ATT_KI);
}

bool imu_pipeline_process(imu_pipeline_t *pipeline,
                          const mpu9250_sample_t *raw,
                          mpu9250_sample_t *out)
{
    if (pipeline == NULL || !pipeline->ready || raw == NULL) {
        return false;
    }

    mpu9250_sample_t filtered;
    mpu9250_sample_t compensated;

    imu_filter_process(raw, &filtered, &pipeline->filter);
    imu_bias_calib_update(&pipeline->bias, &filtered, &compensated);
    imu_attitude_update(&pipeline->attitude, &compensated, pipeline->sample_dt_sec);

    if (out != NULL) {
        *out = compensated;
    }
    return true;
}

bool imu_pipeline_calib_done(const imu_pipeline_t *pipeline)
{
    return (pipeline != NULL) && imu_bias_calib_is_done(&pipeline->bias);
}
