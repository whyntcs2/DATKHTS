#include <stdio.h>

#include "../motion_features.h"

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

int main(void)
{
    KalmanTrack tracks[3] = {{0}};
    TargetFeatureState states[3];
    AI_Features features[3];
    TargetUIData ui;
    float input[AI_FEATURE_COUNT];
    uint32_t i;
    int passed = 1;

    for (i = 0; i < 3; i++) {
        Feature_Reset(&states[i]);
        AI_Features_Reset(&features[i]);
        tracks[i].active = 1;
        tracks[i].initialized = 1;
        tracks[i].dt = 0.1f;
    }

    tracks[0].x = 100.0f;
    tracks[0].y = 200.0f;
    tracks[0].vx = 10.0f;
    tracks[0].vy = -20.0f;
    Feature_Update(&states[0], &tracks[0], &features[0], tracks[0].dt);
    passed &= check(near(features[0].delta_x, 0.0f, 0.001f),
                    "first delta X must be zero");
    passed &= check(near(features[0].ax, 0.0f, 0.001f),
                    "first acceleration X must be zero");

    tracks[0].x = 112.0f;
    tracks[0].y = 195.0f;
    tracks[0].vx = 30.0f;
    tracks[0].vy = -10.0f;
    Feature_Update(&states[0], &tracks[0], &features[0], tracks[0].dt);
    passed &= check(near(features[0].delta_x, 12.0f, 0.001f),
                    "delta X must use filtered position history");
    passed &= check(near(features[0].delta_y, -5.0f, 0.001f),
                    "delta Y must use filtered position history");
    passed &= check(near(features[0].ax, 200.0f, 0.001f),
                    "acceleration X must use Kalman velocity");
    passed &= check(near(features[0].ay, 100.0f, 0.001f),
                    "acceleration Y must use Kalman velocity");

    tracks[1].x = -500.0f;
    tracks[1].y = 900.0f;
    tracks[1].vx = -40.0f;
    tracks[1].vy = 25.0f;
    Feature_Update(&states[1], &tracks[1], &features[1], tracks[1].dt);
    passed &= check(near(features[1].delta_x, 0.0f, 0.001f) &&
                    near(features[1].delta_y, 0.0f, 0.001f),
                    "each target must have independent first-sample history");
    passed &= check(states[0].initialized && states[1].initialized &&
                    !states[2].initialized,
                    "three target histories must be independent");

    tracks[1].vx = 100.0f;
    tracks[1].vy = 100.0f;
    Feature_Update(&states[1], &tracks[1], &features[1], 0.0f);
    passed &= check(near(features[1].ax, 0.0f, 0.001f) &&
                    near(features[1].ay, 0.0f, 0.001f),
                    "zero dt must not divide");

    AI_Features_ToInput(&features[0], input);
    passed &= check(near(input[0], features[0].delta_x, 0.001f) &&
                    near(input[1], features[0].delta_y, 0.001f) &&
                    near(input[2], features[0].vx, 0.001f) &&
                    near(input[3], features[0].vy, 0.001f) &&
                    near(input[4], features[0].ax, 0.001f) &&
                    near(input[5], features[0].ay, 0.001f),
                    "AI input order must be dX,dY,Vx,Vy,Ax,Ay");

    tracks[2].x = 1000.0f;
    tracks[2].y = 1000.0f;
    tracks[2].vx = 300.0f;
    tracks[2].vy = 400.0f;
    tracks[2].missed_frames = 1;
    TargetUI_Update(&ui, &tracks[2]);
    passed &= check(near(ui.distance_m, 1.4142f, 0.002f),
                    "UI distance must use filtered x and y");
    passed &= check(near(ui.speed_m_s, 0.5f, 0.002f),
                    "UI speed must use Kalman vx and vy");
    passed &= check(near(ui.angle_deg, 45.0f, 0.5f),
                    "angle must be atan2(x,y) with positive X positive");
    passed &= check(ui.valid && ui.predicted,
                    "active missed track must be marked predicted");

    tracks[2].x = -1000.0f;
    TargetUI_Update(&ui, &tracks[2]);
    passed &= check(near(ui.angle_deg, -45.0f, 0.5f),
                    "negative X must produce a negative angle");

    tracks[2].active = 0;
    TargetUI_Update(&ui, &tracks[2]);
    passed &= check(!ui.valid && !ui.predicted,
                    "inactive track must reset UI flags");

    Feature_Reset(&states[0]);
    passed &= check(!states[0].initialized,
                    "feature history must reset with a track");

    if (!passed)
        return 1;

    puts("PASS: motion feature and UI calculations");
    return 0;
}
