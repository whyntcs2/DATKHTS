#include <stdio.h>

#include "../motion_features.h"
#include "../target_tracking.h"

static int check(int condition, const char *message)
{
    if (condition)
        return 1;
    printf("FAIL: %s\n", message);
    return 0;
}

static int near(float value, float expected, float tolerance)
{
    float difference = value - expected;

    if (difference < 0.0f)
        difference = -difference;
    return difference <= tolerance;
}

static int test_feature_formulas(void)
{
    TargetFeatureState state = {0};
    AI_Features features;
    uint32_t i;
    int passed = 1;

    state.window_count = AI_WINDOW_FRAMES;
    for (i = 0; i < AI_WINDOW_FRAMES; i++) {
        state.window[i].x = (float)i;
        state.window[i].y = 0.0f;
        state.window[i].vx = (float)i;
        state.window[i].vy = -(float)i;
        state.window[i].speed = (float)i;
        state.window[i].accel = -(float)i;
        state.window[i].dt_valid = 1u;
    }

    passed &= check(AI_ExtractFeatures(&state, &features),
                    "full valid window must produce features");
    passed &= check(near(features.value[AI_FEATURE_DX_MM], 9.0f, 0.0001f),
                    "dx");
    passed &= check(near(features.value[AI_FEATURE_DY_MM], 0.0f, 0.0001f),
                    "dy");
    passed &= check(near(features.value[AI_FEATURE_MEAN_VX_MM_S], 4.5f, 0.0001f),
                    "mean vx");
    passed &= check(near(features.value[AI_FEATURE_MEAN_VY_MM_S], -4.5f, 0.0001f),
                    "mean vy");
    passed &= check(near(features.value[AI_FEATURE_MEAN_SPEED_MM_S], 4.5f, 0.0001f),
                    "mean speed");
    passed &= check(near(features.value[AI_FEATURE_MEDIAN_SPEED_MM_S], 4.5f, 0.0001f),
                    "median speed");
    passed &= check(near(features.value[AI_FEATURE_MAX_SPEED_MM_S], 9.0f, 0.0001f),
                    "max speed");
    passed &= check(near(features.value[AI_FEATURE_P90_SPEED_MM_S], 8.1f, 0.0001f),
                    "NumPy linear p90");
    passed &= check(near(features.value[AI_FEATURE_MEAN_ACCEL_MM_S2], 4.5f, 0.0001f),
                    "mean absolute acceleration");
    passed &= check(near(features.value[AI_FEATURE_MEDIAN_ACCEL_MM_S2], 4.5f, 0.0001f),
                    "median absolute acceleration");
    passed &= check(near(features.value[AI_FEATURE_MAX_ACCEL_MM_S2], 9.0f, 0.0001f),
                    "max absolute acceleration");
    passed &= check(near(features.value[AI_FEATURE_STD_SPEED_MM_S], 2.8722813f, 0.0002f),
                    "population speed standard deviation");
    passed &= check(near(features.value[AI_FEATURE_PATH_LENGTH_MM], 9.0f, 0.0001f),
                    "path length");
    passed &= check(near(features.value[AI_FEATURE_NET_DISPLACEMENT_MM], 9.0f, 0.0001f),
                    "net displacement");
    passed &= check(near(features.value[AI_FEATURE_STRAIGHTNESS], 1.0f, 0.0001f),
                    "straightness");

    for (i = 0; i < AI_WINDOW_FRAMES; i++) {
        state.window[i].x = 3.0f;
        state.window[i].speed = 0.0f;
        state.window[i].accel = 0.0f;
    }
    passed &= check(AI_ExtractFeatures(&state, &features),
                    "stationary window must remain valid");
    passed &= check(features.value[AI_FEATURE_STRAIGHTNESS] == 0.0f,
                    "zero path must not divide by zero");
    return passed;
}

static int test_normalization_and_quantization(void)
{
    static const float mean[AI_FEATURE_COUNT] = {
        113.54660693930299f, 98.46531904071423f,
        138.6185028343803f, 119.56423018250211f,
        669.2732981010357f, 673.2906700809257f,
        847.362353115194f, 823.316435986725f,
        564.3419670043027f, 364.56183641749146f,
        1962.0069664568896f, 134.8163451977064f,
        547.6355889458931f, 522.6601431850764f,
        0.8730364471396381f
    };
    const float input_scale = 0.11223038626702184f;
    AI_Features features;
    float normalized[AI_FEATURE_COUNT];
    int8_t input[AI_FEATURE_COUNT];
    uint32_t i;
    int passed = 1;

    for (i = 0; i < AI_FEATURE_COUNT; i++)
        features.value[i] = mean[i];
    passed &= check(AI_NormalizeFeatures(&features, normalized),
                    "normalization constants must be valid");
    for (i = 0; i < AI_FEATURE_COUNT; i++)
        passed &= check(normalized[i] == 0.0f, "feature mean must normalize to zero");

    for (i = 0; i < AI_FEATURE_COUNT; i++)
        normalized[i] = 0.0f;
    normalized[0] = 0.5f * input_scale;
    normalized[1] = 1.5f * input_scale;
    normalized[2] = 2.5f * input_scale;
    normalized[3] = -0.5f * input_scale;
    normalized[4] = -1.5f * input_scale;
    normalized[5] = 1000.0f;
    normalized[6] = -1000.0f;
    AI_Quantize(normalized, input);
    passed &= check(input[0] == 0 && input[1] == 2 && input[2] == 2,
                    "positive quantization must use nearest-even");
    passed &= check(input[3] == 0 && input[4] == -2,
                    "negative quantization must use nearest-even");
    passed &= check(input[5] == 127 && input[6] == -127,
                    "quantization must saturate to symmetric INT8 range");
    return passed;
}

static int test_ring_and_stride(void)
{
    TargetFeatureState state;
    AI_Features features;
    float normalized[AI_FEATURE_COUNT];
    int8_t input[AI_FEATURE_COUNT];
    AI_TemporalSample previous;
    AI_TemporalSample current;
    uint32_t i;
    uint32_t generated = 0u;
    int passed = 1;

    Feature_Reset(&state);
    for (i = 0; i < 17u; i++) {
        if (AI_Pipeline_Push(&state, (float)(20u * i), (float)(5u * i),
                             (i == 0u) ? 0.0f : 0.1f,
                             &features, normalized, input))
            generated++;
    }
    passed &= check(generated == 2u,
                    "10-frame window with stride 5 must emit twice after 15 finalized samples");
    passed &= check(AI_TemporalCount(&state) == AI_WINDOW_FRAMES,
                    "ring buffer count must stop at window size");
    passed &= check(AI_TemporalGet(&state, 0u, &previous),
                    "ring buffer logical index zero");
    for (i = 1; i < AI_WINDOW_FRAMES; i++) {
        passed &= check(AI_TemporalGet(&state, i, &current),
                        "ring buffer logical index");
        passed &= check(current.x > previous.x,
                        "ring buffer order must remain chronological after wrap");
        previous = current;
    }
    return passed;
}

static int test_tracking_and_reset(void)
{
    KalmanTrack tracks[LD2450_TARGET_COUNT];
    ld2450_frame_t frame = {0};
    TrackingResult result;
    TargetFeatureState pipeline;
    AI_Features features;
    float normalized[AI_FEATURE_COUNT];
    int8_t input[AI_FEATURE_COUNT];
    uint32_t i;
    int passed = 1;

    for (i = 0; i < LD2450_TARGET_COUNT; i++)
        Kalman_Reset(&tracks[i]);
    Kalman_Init(&tracks[0], -500.0f, 1000.0f, 0.1f,
                1000.0f, 2500.0f, 2500.0f);
    Kalman_Init(&tracks[1], 500.0f, 1000.0f, 0.1f,
                1000.0f, 2500.0f, 2500.0f);

    frame.target[0].valid = 1u;
    frame.target[0].x_mm = 520;
    frame.target[0].y_mm = 1000;
    frame.target[1].valid = 1u;
    frame.target[1].x_mm = -480;
    frame.target[1].y_mm = 1000;
    Tracking_Update(&frame, tracks, 0.1f, &result);
    passed &= check(result.detection_for_track[0] == 1 &&
                    result.detection_for_track[1] == 0,
                    "association must preserve identity when LD2450 slots swap");
    passed &= check(result.detection_for_track[0] != result.detection_for_track[1],
                    "one detection must not update two tracks");

    Feature_Reset(&pipeline);
    for (i = 0; i < 4u; i++)
        AI_Pipeline_Push(&pipeline, (float)i, 0.0f,
                         (i == 0u) ? 0.0f : 0.1f,
                         &features, normalized, input);
    frame.target[0].valid = 0u;
    frame.target[1].valid = 0u;
    for (i = 0; i <= TRACKING_MAX_MISSES; i++)
        Tracking_Update(&frame, tracks, 0.1f, &result);
    passed &= check((result.reset_mask & 0x03u) == 0x03u,
                    "tracks must reset after miss threshold");
    if (result.reset_mask & 0x01u)
        Feature_Reset(&pipeline);
    passed &= check(AI_TemporalCount(&pipeline) == 0u &&
                    pipeline.raw_count == 0u,
                    "track reset must clear temporal state");
    return passed;
}

int main(void)
{
    int passed = 1;

    passed &= test_feature_formulas();
    passed &= test_normalization_and_quantization();
    passed &= test_ring_and_stride();
    passed &= test_tracking_and_reset();
    if (!passed)
        return 1;
    puts("PASS: tracking, temporal preprocessing, 15 features, normalization, INT8");
    return 0;
}
