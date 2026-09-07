#include "kalman.h"

#define KALMAN_STATE_SIZE 4u
#define KALMAN_DET_EPSILON 1.0e-6f

static void clear_matrix(float matrix[4][4])
{
    uint32_t row;
    uint32_t column;

    for (row = 0; row < KALMAN_STATE_SIZE; row++)
        for (column = 0; column < KALMAN_STATE_SIZE; column++)
            matrix[row][column] = 0.0f;
}

void Kalman_Reset(KalmanTrack *kf)
{
    clear_matrix(kf->P);
    kf->x = 0.0f;
    kf->y = 0.0f;
    kf->vx = 0.0f;
    kf->vy = 0.0f;
    kf->dt = 0.0f;
    kf->sigma_a = 0.0f;
    kf->r_x = 0.0f;
    kf->r_y = 0.0f;
    kf->initialized = 0;
    kf->active = 0;
    kf->missed_frames = 0;
}

void Kalman_Init(KalmanTrack *kf,
                 float x,
                 float y,
                 float dt,
                 float sigma_a,
                 float r_x,
                 float r_y)
{
    float velocity_variance;

    Kalman_Reset(kf);

    kf->x = x;
    kf->y = y;
    kf->dt = dt;
    kf->sigma_a = sigma_a;
    kf->r_x = r_x;
    kf->r_y = r_y;

    /* Position starts at the first measurement. Velocity is unknown. */
    velocity_variance = sigma_a * sigma_a;
    if (velocity_variance < 1.0f)
        velocity_variance = 1.0f;

    kf->P[0][0] = (r_x > 0.0f) ? r_x : 1.0f;
    kf->P[1][1] = (r_y > 0.0f) ? r_y : 1.0f;
    kf->P[2][2] = velocity_variance;
    kf->P[3][3] = velocity_variance;

    kf->initialized = 1;
    kf->active = 1;
}

void Kalman_SetDt(KalmanTrack *kf, float dt)
{
    if (dt > 0.0f)
        kf->dt = dt;
}

void Kalman_Predict(KalmanTrack *kf)
{
    float F[4][4] = {
        {1.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f}
    };
    float FP[4][4];
    float predicted_P[4][4];
    float dt2;
    float dt3;
    float dt4;
    float acceleration_variance;
    float q_position;
    float q_cross;
    float q_velocity;
    float sum;
    uint32_t row;
    uint32_t column;
    uint32_t index;

    if (!kf->initialized)
        return;

    F[0][2] = kf->dt;
    F[1][3] = kf->dt;

    kf->x += kf->vx * kf->dt;
    kf->y += kf->vy * kf->dt;

    for (row = 0; row < KALMAN_STATE_SIZE; row++) {
        for (column = 0; column < KALMAN_STATE_SIZE; column++) {
            sum = 0.0f;
            for (index = 0; index < KALMAN_STATE_SIZE; index++)
                sum += F[row][index] * kf->P[index][column];
            FP[row][column] = sum;
        }
    }

    for (row = 0; row < KALMAN_STATE_SIZE; row++) {
        for (column = 0; column < KALMAN_STATE_SIZE; column++) {
            sum = 0.0f;
            for (index = 0; index < KALMAN_STATE_SIZE; index++)
                sum += FP[row][index] * F[column][index];
            predicted_P[row][column] = sum;
        }
    }

    dt2 = kf->dt * kf->dt;
    dt3 = dt2 * kf->dt;
    dt4 = dt2 * dt2;
    acceleration_variance = kf->sigma_a * kf->sigma_a;
    q_position = acceleration_variance * dt4 * 0.25f;
    q_cross = acceleration_variance * dt3 * 0.5f;
    q_velocity = acceleration_variance * dt2;

    predicted_P[0][0] += q_position;
    predicted_P[1][1] += q_position;
    predicted_P[0][2] += q_cross;
    predicted_P[2][0] += q_cross;
    predicted_P[1][3] += q_cross;
    predicted_P[3][1] += q_cross;
    predicted_P[2][2] += q_velocity;
    predicted_P[3][3] += q_velocity;

    for (row = 0; row < KALMAN_STATE_SIZE; row++)
        for (column = 0; column < KALMAN_STATE_SIZE; column++)
            kf->P[row][column] = predicted_P[row][column];
}

void Kalman_Update(KalmanTrack *kf, float x_measured, float y_measured)
{
    float innovation_x;
    float innovation_y;
    float s00;
    float s01;
    float s10;
    float s11;
    float determinant;
    float inverse_determinant;
    float gain[4][2];
    float corrected_P[4][4];
    float state[4];
    float covariance_average;
    uint32_t row;
    uint32_t column;

    if (!kf->initialized)
        return;

    innovation_x = x_measured - kf->x;
    innovation_y = y_measured - kf->y;

    /* H selects x and y, so S = HPH' + R is the top-left 2x2 block. */
    s00 = kf->P[0][0] + kf->r_x;
    s01 = kf->P[0][1];
    s10 = kf->P[1][0];
    s11 = kf->P[1][1] + kf->r_y;
    determinant = s00 * s11 - s01 * s10;

    if (determinant <= KALMAN_DET_EPSILON)
        return;

    inverse_determinant = 1.0f / determinant;
    for (row = 0; row < KALMAN_STATE_SIZE; row++) {
        gain[row][0] = (kf->P[row][0] * s11 -
                        kf->P[row][1] * s10) * inverse_determinant;
        gain[row][1] = (-kf->P[row][0] * s01 +
                         kf->P[row][1] * s00) * inverse_determinant;
    }

    state[0] = kf->x;
    state[1] = kf->y;
    state[2] = kf->vx;
    state[3] = kf->vy;
    for (row = 0; row < KALMAN_STATE_SIZE; row++)
        state[row] += gain[row][0] * innovation_x +
                      gain[row][1] * innovation_y;

    /* P = (I - KH)P; H only contains the first two identity rows. */
    for (row = 0; row < KALMAN_STATE_SIZE; row++) {
        for (column = 0; column < KALMAN_STATE_SIZE; column++) {
            corrected_P[row][column] = kf->P[row][column] -
                                       gain[row][0] * kf->P[0][column] -
                                       gain[row][1] * kf->P[1][column];
        }
    }

    /* Limit small floating-point asymmetry accumulated by repeated updates. */
    for (row = 0; row < KALMAN_STATE_SIZE; row++) {
        for (column = row + 1; column < KALMAN_STATE_SIZE; column++) {
            covariance_average = 0.5f *
                (corrected_P[row][column] + corrected_P[column][row]);
            corrected_P[row][column] = covariance_average;
            corrected_P[column][row] = covariance_average;
        }
    }

    kf->x = state[0];
    kf->y = state[1];
    kf->vx = state[2];
    kf->vy = state[3];
    for (row = 0; row < KALMAN_STATE_SIZE; row++)
        for (column = 0; column < KALMAN_STATE_SIZE; column++)
            kf->P[row][column] = corrected_P[row][column];
}
