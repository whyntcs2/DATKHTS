#include "motion_features.h"

#define FEATURE_MIN_DT_S 1.0e-6f
#define PI_F             3.14159265358979323846f
#define HALF_PI_F        1.57079632679489661923f
#define RAD_TO_DEG_F    57.2957795130823208768f

static float square_root(float value)
{
    union {
        float value;
        uint32_t bits;
    } estimate;
    uint32_t iteration;

    if (value <= 0.0f)
        return 0.0f;

    /* IEEE-754 initial estimate followed by Newton-Raphson refinement. */
    estimate.value = value;
    estimate.bits = (estimate.bits >> 1) + 0x1fc00000u;
    for (iteration = 0; iteration < 3; iteration++)
        estimate.value = 0.5f * (estimate.value + value / estimate.value);

    return estimate.value;
}

static float atan_approx(float value)
{
    float squared = value * value;

    if (value > 1.0f)
        return HALF_PI_F - value / (squared + 0.280872f);
    if (value < -1.0f)
        return -HALF_PI_F - value / (squared + 0.280872f);

    return value / (1.0f + 0.280872f * squared);
}

/* Equivalent to atan2f(x, y): zero degrees points along positive Y. */
static float target_angle_degrees(float x, float y)
{
    float angle_rad;

    if (y > 0.0f) {
        angle_rad = atan_approx(x / y);
    } else if (y < 0.0f) {
        angle_rad = atan_approx(x / y);
        angle_rad += (x >= 0.0f) ? PI_F : -PI_F;
    } else if (x > 0.0f) {
        angle_rad = HALF_PI_F;
    } else if (x < 0.0f) {
        angle_rad = -HALF_PI_F;
    } else {
        angle_rad = 0.0f;
    }

    return angle_rad * RAD_TO_DEG_F;
}

void Feature_Reset(TargetFeatureState *state)
{
    state->prev_x = 0.0f;
    state->prev_y = 0.0f;
    state->prev_vx = 0.0f;
    state->prev_vy = 0.0f;
    state->initialized = 0;
}

void AI_Features_Reset(AI_Features *feature)
{
    feature->delta_x = 0.0f;
    feature->delta_y = 0.0f;
    feature->vx = 0.0f;
    feature->vy = 0.0f;
    feature->ax = 0.0f;
    feature->ay = 0.0f;
}

void Feature_Update(TargetFeatureState *state,
                    const KalmanTrack *kf,
                    AI_Features *feature,
                    float dt)
{
    if (!state->initialized) {
        feature->delta_x = 0.0f;
        feature->delta_y = 0.0f;
        feature->vx = kf->vx;
        feature->vy = kf->vy;
        feature->ax = 0.0f;
        feature->ay = 0.0f;
        state->initialized = 1;
    } else {
        feature->delta_x = kf->x - state->prev_x;
        feature->delta_y = kf->y - state->prev_y;
        feature->vx = kf->vx;
        feature->vy = kf->vy;

        if (dt > FEATURE_MIN_DT_S) {
            feature->ax = (kf->vx - state->prev_vx) / dt;
            feature->ay = (kf->vy - state->prev_vy) / dt;
        } else {
            feature->ax = 0.0f;
            feature->ay = 0.0f;
        }
    }

    state->prev_x = kf->x;
    state->prev_y = kf->y;
    state->prev_vx = kf->vx;
    state->prev_vy = kf->vy;
}

void AI_Features_ToInput(const AI_Features *feature,
                         float input[AI_FEATURE_COUNT])
{
    /* Fixed training/inference order: dX, dY, Vx, Vy, Ax, Ay. */
    input[AI_FEATURE_DELTA_X] = feature->delta_x;
    input[AI_FEATURE_DELTA_Y] = feature->delta_y;
    input[AI_FEATURE_VX] = feature->vx;
    input[AI_FEATURE_VY] = feature->vy;
    input[AI_FEATURE_AX] = feature->ax;
    input[AI_FEATURE_AY] = feature->ay;

    /*
     * TODO: Apply dataset-derived normalization and the trained model's
     * INT8 scale/zero-point before passing these values to the MLP.
     * TODO: Evaluate temporal-window aggregation, especially for Falling.
     */
}

void TargetUI_Reset(TargetUIData *ui)
{
    ui->distance_m = 0.0f;
    ui->speed_m_s = 0.0f;
    ui->angle_deg = 0.0f;
    ui->valid = 0;
    ui->predicted = 0;
}

void TargetUI_Update(TargetUIData *ui, const KalmanTrack *kf)
{
    if (!kf->active) {
        TargetUI_Reset(ui);
        return;
    }

    ui->distance_m = square_root(kf->x * kf->x + kf->y * kf->y) /
                     1000.0f;
    ui->speed_m_s = square_root(kf->vx * kf->vx + kf->vy * kf->vy) /
                    1000.0f;
    ui->angle_deg = target_angle_degrees(kf->x, kf->y);
    ui->valid = 1;
    ui->predicted = (kf->missed_frames != 0);
}
