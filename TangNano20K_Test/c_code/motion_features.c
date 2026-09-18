#include "motion_features.h"

#define FEATURE_MIN_DT_S 1.0e-6f
#define PI_F             3.14159265358979323846f
#define HALF_PI_F        1.57079632679489661923f
#define RAD_TO_DEG_F    57.2957795130823208768f
#define AI_INPUT_SCALE   0.11223038626702184f

/* model_output/normalization.json, fitted on the session-level training split. */
static const float normalization_mean[AI_FEATURE_COUNT] = {
    113.54660693930299f, 98.46531904071423f,
    138.6185028343803f, 119.56423018250211f,
    669.2732981010357f, 673.2906700809257f,
    847.362353115194f, 823.316435986725f,
    564.3419670043027f, 364.56183641749146f,
    1962.0069664568896f, 134.8163451977064f,
    547.6355889458931f, 522.6601431850764f,
    0.8730364471396381f
};

static const float normalization_scale[AI_FEATURE_COUNT] = {
    405.96704554649125f, 660.9435186057934f,
    494.09257042972337f, 808.1837540863972f,
    714.6272595935791f, 735.3799481756146f,
    878.662693984032f, 861.6466715701774f,
    869.1652670433787f, 573.3721088065495f,
    4560.466727880648f, 219.57498789631282f,
    593.8248381984563f, 592.5117742152702f,
    0.19097693355711687f
};

static float absolute_value(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float square_root(float value)
{
    union {
        float value;
        uint32_t bits;
    } estimate;
    uint32_t iteration;

    if (value <= 0.0f)
        return 0.0f;

    estimate.value = value;
    estimate.bits = (estimate.bits >> 1) + 0x1fc00000u;
    for (iteration = 0; iteration < 4; iteration++)
        estimate.value = 0.5f * (estimate.value + value / estimate.value);

    return estimate.value;
}

static void sort_values(float *values, uint32_t count)
{
    uint32_t i;

    for (i = 1; i < count; i++) {
        float value = values[i];
        uint32_t position = i;

        while ((position != 0u) && (values[position - 1u] > value)) {
            values[position] = values[position - 1u];
            position--;
        }
        values[position] = value;
    }
}

static float median(float *values, uint32_t count)
{
    sort_values(values, count);
    if (count & 1u)
        return values[count / 2u];
    return 0.5f * (values[count / 2u - 1u] + values[count / 2u]);
}

static const AI_PendingSample *pending_at(const TargetFeatureState *state,
                                          uint32_t logical_index)
{
    uint32_t physical = (state->pending_start + logical_index) %
                        AI_SPEED_MEDIAN_WINDOW;
    return &state->pending[physical];
}

static void pending_push(TargetFeatureState *state,
                         const AI_PendingSample *sample)
{
    uint32_t physical;

    if (state->pending_count < AI_SPEED_MEDIAN_WINDOW) {
        physical = (state->pending_start + state->pending_count) %
                   AI_SPEED_MEDIAN_WINDOW;
        state->pending_count++;
    } else {
        physical = state->pending_start;
        state->pending_start = (state->pending_start + 1u) %
                               AI_SPEED_MEDIAN_WINDOW;
    }
    state->pending[physical] = *sample;
}

static void temporal_push(TargetFeatureState *state,
                          const AI_TemporalSample *sample)
{
    uint32_t physical;

    if (state->window_count < AI_WINDOW_FRAMES) {
        physical = (state->window_start + state->window_count) %
                   AI_WINDOW_FRAMES;
        state->window_count++;
    } else {
        physical = state->window_start;
        state->window_start = (state->window_start + 1u) % AI_WINDOW_FRAMES;
    }
    state->window[physical] = *sample;
    state->finalized_count++;
}

void Feature_Reset(TargetFeatureState *state)
{
    state->ema_x = 0.0f;
    state->ema_y = 0.0f;
    state->previous_speed = 0.0f;
    state->raw_count = 0u;
    state->finalized_count = 0u;
    state->pending_start = 0u;
    state->pending_count = 0u;
    state->window_start = 0u;
    state->window_count = 0u;
    state->ema_initialized = 0u;
    state->speed_initialized = 0u;
}

void AI_Features_Reset(AI_Features *feature)
{
    uint32_t i;

    for (i = 0; i < AI_FEATURE_COUNT; i++)
        feature->value[i] = 0.0f;
}

uint32_t AI_TemporalCount(const TargetFeatureState *state)
{
    return state->window_count;
}

int AI_TemporalGet(const TargetFeatureState *state,
                   uint32_t logical_index,
                   AI_TemporalSample *sample)
{
    uint32_t physical;

    if (logical_index >= state->window_count)
        return 0;

    physical = (state->window_start + logical_index) % AI_WINDOW_FRAMES;
    *sample = state->window[physical];
    return 1;
}

int AI_ExtractFeatures(const TargetFeatureState *state,
                       AI_Features *features)
{
    AI_TemporalSample samples[AI_WINDOW_FRAMES];
    float sorted_speed[AI_WINDOW_FRAMES];
    float sorted_accel[AI_WINDOW_FRAMES];
    float sum_vx = 0.0f;
    float sum_vy = 0.0f;
    float sum_speed = 0.0f;
    float sum_accel = 0.0f;
    float sum_speed_squared = 0.0f;
    float path_length = 0.0f;
    float max_speed = 0.0f;
    float max_accel = 0.0f;
    float dx;
    float dy;
    float net_displacement;
    float mean_speed;
    float variance;
    uint32_t i;

    if (state->window_count != AI_WINDOW_FRAMES)
        return 0;

    for (i = 0; i < AI_WINDOW_FRAMES; i++) {
        if (!AI_TemporalGet(state, i, &samples[i]))
            return 0;
        if ((i != 0u) && !samples[i].dt_valid)
            return 0;
    }

    for (i = 0; i < AI_WINDOW_FRAMES; i++) {
        float abs_accel = absolute_value(samples[i].accel);

        sum_vx += samples[i].vx;
        sum_vy += samples[i].vy;
        sum_speed += samples[i].speed;
        sum_speed_squared += samples[i].speed * samples[i].speed;
        sum_accel += abs_accel;
        sorted_speed[i] = samples[i].speed;
        sorted_accel[i] = abs_accel;
        if ((i == 0u) || (samples[i].speed > max_speed))
            max_speed = samples[i].speed;
        if ((i == 0u) || (abs_accel > max_accel))
            max_accel = abs_accel;
        if (i != 0u) {
            float step_x = samples[i].x - samples[i - 1u].x;
            float step_y = samples[i].y - samples[i - 1u].y;
            path_length += square_root(step_x * step_x + step_y * step_y);
        }
    }

    dx = samples[AI_WINDOW_FRAMES - 1u].x - samples[0].x;
    dy = samples[AI_WINDOW_FRAMES - 1u].y - samples[0].y;
    net_displacement = square_root(dx * dx + dy * dy);
    mean_speed = sum_speed / (float)AI_WINDOW_FRAMES;
    variance = sum_speed_squared / (float)AI_WINDOW_FRAMES -
               mean_speed * mean_speed;
    if (variance < 0.0f)
        variance = 0.0f;

    features->value[AI_FEATURE_DX_MM] = dx;
    features->value[AI_FEATURE_DY_MM] = dy;
    features->value[AI_FEATURE_MEAN_VX_MM_S] =
        sum_vx / (float)AI_WINDOW_FRAMES;
    features->value[AI_FEATURE_MEAN_VY_MM_S] =
        sum_vy / (float)AI_WINDOW_FRAMES;
    features->value[AI_FEATURE_MEAN_SPEED_MM_S] = mean_speed;
    features->value[AI_FEATURE_MEDIAN_SPEED_MM_S] =
        median(sorted_speed, AI_WINDOW_FRAMES);
    features->value[AI_FEATURE_MAX_SPEED_MM_S] = max_speed;

    sort_values(sorted_speed, AI_WINDOW_FRAMES);
    features->value[AI_FEATURE_P90_SPEED_MM_S] =
        sorted_speed[8] + 0.1f * (sorted_speed[9] - sorted_speed[8]);

    features->value[AI_FEATURE_MEAN_ACCEL_MM_S2] =
        sum_accel / (float)AI_WINDOW_FRAMES;
    features->value[AI_FEATURE_MEDIAN_ACCEL_MM_S2] =
        median(sorted_accel, AI_WINDOW_FRAMES);
    features->value[AI_FEATURE_MAX_ACCEL_MM_S2] = max_accel;
    features->value[AI_FEATURE_STD_SPEED_MM_S] = square_root(variance);
    features->value[AI_FEATURE_PATH_LENGTH_MM] = path_length;
    features->value[AI_FEATURE_NET_DISPLACEMENT_MM] = net_displacement;
    features->value[AI_FEATURE_STRAIGHTNESS] =
        (path_length > FEATURE_MIN_DT_S) ? net_displacement / path_length : 0.0f;
    return 1;
}

int AI_NormalizeFeatures(const AI_Features *features,
                         float normalized[AI_FEATURE_COUNT])
{
    uint32_t i;

    for (i = 0; i < AI_FEATURE_COUNT; i++) {
        if (normalization_scale[i] <= 0.0f)
            return 0;
        normalized[i] = (features->value[i] - normalization_mean[i]) /
                        normalization_scale[i];
    }
    return 1;
}

static int32_t round_nearest_even(float value)
{
    int32_t whole;
    float fraction;

    if (value < 0.0f)
        return -round_nearest_even(-value);

    whole = (int32_t)value;
    fraction = value - (float)whole;
    if ((fraction > 0.5f) || ((fraction == 0.5f) && (whole & 1)))
        whole++;
    return whole;
}

void AI_Quantize(const float normalized[AI_FEATURE_COUNT],
                 int8_t ai_input[AI_FEATURE_COUNT])
{
    uint32_t i;

    for (i = 0; i < AI_FEATURE_COUNT; i++) {
        float scaled = normalized[i] / AI_INPUT_SCALE;
        int32_t quantized;

        if (scaled >= 127.0f)
            quantized = 127;
        else if (scaled <= -127.0f)
            quantized = -127;
        else
            quantized = round_nearest_even(scaled);
        ai_input[i] = (int8_t)quantized;
    }
}

int AI_Pipeline_Push(TargetFeatureState *state,
                     float x_mm,
                     float y_mm,
                     float dt_s,
                     AI_Features *features,
                     float normalized[AI_FEATURE_COUNT],
                     int8_t ai_input[AI_FEATURE_COUNT])
{
    AI_PendingSample pending;
    AI_TemporalSample finalized;
    float speeds[AI_SPEED_MEDIAN_WINDOW];
    uint32_t candidate_index;
    uint32_t i;

    if (!state->ema_initialized) {
        state->ema_x = x_mm;
        state->ema_y = y_mm;
        state->ema_initialized = 1u;
        pending.vx = 0.0f;
        pending.vy = 0.0f;
        pending.speed_raw = 0.0f;
        pending.dt_valid = 1u;
        pending.dt = 0.0f;
    } else {
        float previous_x = state->ema_x;
        float previous_y = state->ema_y;

        state->ema_x = AI_EMA_ALPHA * x_mm +
                       (1.0f - AI_EMA_ALPHA) * state->ema_x;
        state->ema_y = AI_EMA_ALPHA * y_mm +
                       (1.0f - AI_EMA_ALPHA) * state->ema_y;
        pending.dt_valid = (dt_s > 0.0f) && (dt_s <= AI_MAX_DT_S);
        pending.dt = dt_s;
        if (pending.dt_valid) {
            pending.vx = (state->ema_x - previous_x) / dt_s;
            pending.vy = (state->ema_y - previous_y) / dt_s;
            pending.speed_raw = square_root(pending.vx * pending.vx +
                                            pending.vy * pending.vy);
        } else {
            pending.vx = 0.0f;
            pending.vy = 0.0f;
            pending.speed_raw = 0.0f;
        }
    }

    pending.x = state->ema_x;
    pending.y = state->ema_y;
    pending_push(state, &pending);
    state->raw_count++;

    /* pandas rolling(center=True, window=5): two future samples are required. */
    if (state->pending_count < 3u)
        return 0;

    candidate_index = state->pending_count - 3u;
    pending = *pending_at(state, candidate_index);
    for (i = 0; i < state->pending_count; i++)
        speeds[i] = pending_at(state, i)->speed_raw;

    finalized.x = pending.x;
    finalized.y = pending.y;
    finalized.vx = pending.vx;
    finalized.vy = pending.vy;
    finalized.speed = median(speeds, state->pending_count);
    finalized.dt_valid = pending.dt_valid;
    if (!state->speed_initialized) {
        finalized.accel = 0.0f;
        state->speed_initialized = 1u;
    } else if (pending.dt_valid) {
        finalized.accel = (finalized.speed - state->previous_speed) / pending.dt;
    } else {
        finalized.accel = 0.0f;
    }
    state->previous_speed = finalized.speed;
    temporal_push(state, &finalized);

    if ((state->finalized_count < AI_WINDOW_FRAMES) ||
        (((state->finalized_count - AI_WINDOW_FRAMES) % AI_WINDOW_STRIDE) != 0u))
        return 0;
    if (!AI_ExtractFeatures(state, features))
        return 0;
    if (!AI_NormalizeFeatures(features, normalized))
        return 0;
    AI_Quantize(normalized, ai_input);
    return 1;
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

    ui->distance_m = square_root(kf->x * kf->x + kf->y * kf->y) / 1000.0f;
    ui->speed_m_s = square_root(kf->vx * kf->vx + kf->vy * kf->vy) / 1000.0f;
    ui->angle_deg = target_angle_degrees(kf->x, kf->y);
    ui->valid = 1;
    ui->predicted = (kf->missed_frames != 0);
}
