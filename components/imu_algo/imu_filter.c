#include "imu_filter.h"

#include <math.h>
#include <string.h>

static void biquad_reset(imu_biquad_state_t *state)
{
    state->x1 = 0.0f;
    state->x2 = 0.0f;
    state->y1 = 0.0f;
    state->y2 = 0.0f;
    state->warmup_count = 0u;
}

static void biquad_design_lp2(float sample_hz, float cutoff_hz, imu_biquad_coeff_t *coeff)
{
    if (cutoff_hz <= 0.0f || sample_hz <= 0.0f) {
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

static void biquad_init(imu_biquad_state_t *state, uint16_t warmup_limit)
{
    biquad_reset(state);
    state->warmup_limit = warmup_limit;
}

static float biquad_process(float input,
                            imu_biquad_state_t *state,
                            const imu_biquad_coeff_t *coeff)
{
    float y;

    if (state->warmup_count < state->warmup_limit) {
        y = input;
        state->warmup_count++;
    } else {
        y = coeff->b0 * input
           + coeff->b1 * state->x1
           + coeff->b2 * state->x2
           - coeff->a1 * state->y1
           - coeff->a2 * state->y2;
    }

    state->x2 = state->x1;
    state->x1 = input;
    state->y2 = state->y1;
    state->y1 = y;
    return y;
}

void imu_filter_init(imu_filter_group_t *filter,
                     float sample_hz,
                     float acc_cutoff_hz,
                     float gyro_cutoff_hz,
                     uint16_t warmup_limit)
{
    if (filter == NULL) {
        return;
    }

    memset(filter, 0, sizeof(*filter));
    for (int i = 0; i < 3; i++) {
        biquad_init(&filter->accel_state[i], warmup_limit);
        biquad_design_lp2(sample_hz, acc_cutoff_hz, &filter->accel_coeff[i]);

        biquad_init(&filter->gyro_state[i], warmup_limit);
        biquad_design_lp2(sample_hz, gyro_cutoff_hz, &filter->gyro_coeff[i]);
    }
}

void imu_filter_process(const mpu9250_sample_t *input,
                        mpu9250_sample_t *output,
                        imu_filter_group_t *filter)
{
    if (input == NULL || output == NULL || filter == NULL) {
        return;
    }

    output->acc_x = biquad_process(input->acc_x, &filter->accel_state[0], &filter->accel_coeff[0]);
    output->acc_y = biquad_process(input->acc_y, &filter->accel_state[1], &filter->accel_coeff[1]);
    output->acc_z = biquad_process(input->acc_z, &filter->accel_state[2], &filter->accel_coeff[2]);

    output->gyro_x = biquad_process(input->gyro_x, &filter->gyro_state[0], &filter->gyro_coeff[0]);
    output->gyro_y = biquad_process(input->gyro_y, &filter->gyro_state[1], &filter->gyro_coeff[1]);
    output->gyro_z = biquad_process(input->gyro_z, &filter->gyro_state[2], &filter->gyro_coeff[2]);

    /* 温度与磁场不滤波，直接透传 */
    output->temp_c = input->temp_c;
    output->mag_x = input->mag_x;
    output->mag_y = input->mag_y;
    output->mag_z = input->mag_z;
}
