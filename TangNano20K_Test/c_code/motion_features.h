#ifndef MOTION_FEATURES_H
#define MOTION_FEATURES_H

#include "std_int_types.h"
#include "kalman.h"

#define AI_FEATURE_COUNT            15u
#define AI_WINDOW_FRAMES            10u
#define AI_WINDOW_STRIDE             5u
#define AI_SPEED_MEDIAN_WINDOW       5u
#define AI_EMA_ALPHA              0.35f
#define AI_MAX_DT_S               0.25f

enum {
    AI_FEATURE_DX_MM = 0,
    AI_FEATURE_DY_MM,
    AI_FEATURE_MEAN_VX_MM_S,
    AI_FEATURE_MEAN_VY_MM_S,
    AI_FEATURE_MEAN_SPEED_MM_S,
    AI_FEATURE_MEDIAN_SPEED_MM_S,
    AI_FEATURE_MAX_SPEED_MM_S,
    AI_FEATURE_P90_SPEED_MM_S,
    AI_FEATURE_MEAN_ACCEL_MM_S2,
    AI_FEATURE_MEDIAN_ACCEL_MM_S2,
    AI_FEATURE_MAX_ACCEL_MM_S2,
    AI_FEATURE_STD_SPEED_MM_S,
    AI_FEATURE_PATH_LENGTH_MM,
    AI_FEATURE_NET_DISPLACEMENT_MM,
    AI_FEATURE_STRAIGHTNESS
};

typedef struct
{
    float x;
    float y;
    float vx;
    float vy;
    float speed;
    float accel;
    uint8_t dt_valid;
} AI_TemporalSample;

typedef struct
{
    float x;
    float y;
    float vx;
    float vy;
    float speed_raw;
    float dt;
    uint8_t dt_valid;
} AI_PendingSample;

typedef struct
{
    AI_PendingSample pending[AI_SPEED_MEDIAN_WINDOW];
    AI_TemporalSample window[AI_WINDOW_FRAMES];
    float ema_x;
    float ema_y;
    float previous_speed;
    uint32_t raw_count;
    uint32_t finalized_count;
    uint8_t pending_start;
    uint8_t pending_count;
    uint8_t window_start;
    uint8_t window_count;
    uint8_t ema_initialized;
    uint8_t speed_initialized;
} TargetFeatureState;

typedef struct
{
    float value[AI_FEATURE_COUNT];
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
uint32_t AI_TemporalCount(const TargetFeatureState *state);
int AI_TemporalGet(const TargetFeatureState *state,
                   uint32_t logical_index,
                   AI_TemporalSample *sample);
int AI_ExtractFeatures(const TargetFeatureState *state,
                       AI_Features *features);
int AI_Pipeline_Push(TargetFeatureState *state,
                     float x_mm,
                     float y_mm,
                     float dt_s,
                     AI_Features *features,
                     float normalized[AI_FEATURE_COUNT],
                     int8_t ai_input[AI_FEATURE_COUNT]);
int AI_NormalizeFeatures(const AI_Features *features,
                         float normalized[AI_FEATURE_COUNT]);
void AI_Quantize(const float normalized[AI_FEATURE_COUNT],
                 int8_t ai_input[AI_FEATURE_COUNT]);

void TargetUI_Reset(TargetUIData *ui);
void TargetUI_Update(TargetUIData *ui, const KalmanTrack *kf);

#endif
