#ifndef MOTION_FEATURES_H
#define MOTION_FEATURES_H

#include "std_int_types.h"
#include "kalman.h"

#define AI_FEATURE_COUNT   6u
#define AI_FEATURE_DELTA_X 0u
#define AI_FEATURE_DELTA_Y 1u
#define AI_FEATURE_VX      2u
#define AI_FEATURE_VY      3u
#define AI_FEATURE_AX      4u
#define AI_FEATURE_AY      5u

typedef struct
{
    float prev_x;
    float prev_y;
    float prev_vx;
    float prev_vy;
    uint8_t initialized;
} TargetFeatureState;

typedef struct
{
    float delta_x;
    float delta_y;
    float vx;
    float vy;
    float ax;
    float ay;
} AI_Features;

typedef struct
{
    float distance_m;
    float speed_m_s;
    float angle_deg;
    uint8_t valid;
    uint8_t predicted;
} TargetUIData;

void Feature_Reset(TargetFeatureState *state);
void AI_Features_Reset(AI_Features *feature);
void Feature_Update(TargetFeatureState *state,
                    const KalmanTrack *kf,
                    AI_Features *feature,
                    float dt);
void AI_Features_ToInput(const AI_Features *feature,
                         float input[AI_FEATURE_COUNT]);

void TargetUI_Reset(TargetUIData *ui);
void TargetUI_Update(TargetUIData *ui, const KalmanTrack *kf);

#endif
