#ifndef KALMAN_H
#define KALMAN_H

#include "std_int_types.h"

typedef struct
{
    float x;
    float y;
    float vx;
    float vy;

    float P[4][4];

    float dt;
    float sigma_a;
    float r_x;
    float r_y;

    uint8_t initialized;
    uint8_t active;
    uint8_t missed_frames;
} KalmanTrack;

void Kalman_Init(KalmanTrack *kf,
                 float x,
                 float y,
                 float dt,
                 float sigma_a,
                 float r_x,
                 float r_y);

void Kalman_SetDt(KalmanTrack *kf, float dt);
void Kalman_Predict(KalmanTrack *kf);
void Kalman_Update(KalmanTrack *kf, float x_measured, float y_measured);
void Kalman_Reset(KalmanTrack *kf);

#endif
