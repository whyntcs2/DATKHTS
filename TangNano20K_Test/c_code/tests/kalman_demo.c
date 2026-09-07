#include <stdio.h>

#include "../kalman.h"

#define TRACK_COUNT 3u
#define FRAME_COUNT 4u
#define MAX_MISSED_FRAMES 5u

#define TEST_DT        0.1f
#define TEST_SIGMA_A 1000.0f
#define TEST_R_X     2500.0f
#define TEST_R_Y     2500.0f

typedef struct {
    uint8_t valid;
    float x;
    float y;
} SampleTarget;

static const SampleTarget samples[FRAME_COUNT][TRACK_COUNT] = {
    {{1, -112.0f, 179.0f}, {1, 464.0f, 336.0f}, {0, 0.0f, 0.0f}},
    {{1,  -39.0f, 131.0f}, {1, 461.0f, 333.0f}, {0, 0.0f, 0.0f}},
    {{1,   21.0f,  91.0f}, {1, 458.0f, 331.0f}, {0, 0.0f, 0.0f}},
    {{1,   72.0f,  58.0f}, {0,   0.0f,   0.0f}, {0, 0.0f, 0.0f}}
};

static int check(int condition, const char *message)
{
    if (condition)
        return 1;

    printf("FAIL: %s\n", message);
    return 0;
}

int main(void)
{
    KalmanTrack tracks[TRACK_COUNT];
    KalmanTrack missed_track;
    uint32_t frame;
    uint32_t target;
    int passed = 1;

    for (target = 0; target < TRACK_COUNT; target++)
        Kalman_Reset(&tracks[target]);

    for (frame = 0; frame < FRAME_COUNT; frame++) {
        printf("Frame %lu:\n", (unsigned long)(frame + 1));

        for (target = 0; target < TRACK_COUNT; target++) {
            const SampleTarget *measurement = &samples[frame][target];

            if (measurement->valid) {
                if (!tracks[target].initialized) {
                    Kalman_Init(&tracks[target], measurement->x, measurement->y,
                                TEST_DT, TEST_SIGMA_A, TEST_R_X, TEST_R_Y);
                } else {
                    Kalman_Predict(&tracks[target]);
                    Kalman_Update(&tracks[target], measurement->x, measurement->y);
                }
                tracks[target].active = 1;
                tracks[target].missed_frames = 0;

                printf("T%lu RAW: X=%.0f Y=%.0f\n",
                       (unsigned long)(target + 1),
                       (double)measurement->x,
                       (double)measurement->y);
            } else if (tracks[target].initialized) {
                Kalman_Predict(&tracks[target]);
                tracks[target].missed_frames++;
                if (tracks[target].missed_frames > MAX_MISSED_FRAMES)
                    Kalman_Reset(&tracks[target]);
            }

            if (tracks[target].active) {
                printf("T%lu KF : X=%.2f Y=%.2f VX=%.2f VY=%.2f%s\n",
                       (unsigned long)(target + 1),
                       (double)tracks[target].x,
                       (double)tracks[target].y,
                       (double)tracks[target].vx,
                       (double)tracks[target].vy,
                       tracks[target].missed_frames ? " (predicted)" : "");
            } else {
                printf("T%lu: not detected\n", (unsigned long)(target + 1));
            }
        }
        putchar('\n');
    }

    passed &= check(tracks[0].initialized, "T1 should remain initialized");
    passed &= check(tracks[1].missed_frames == 1,
                    "T2 should be predicted for one missed frame");
    passed &= check(!tracks[2].initialized,
                    "T3 should remain uninitialized without measurements");

    missed_track = tracks[1];
    while (missed_track.initialized) {
        Kalman_Predict(&missed_track);
        missed_track.missed_frames++;
        if (missed_track.missed_frames > MAX_MISSED_FRAMES)
            Kalman_Reset(&missed_track);
    }
    passed &= check(!missed_track.active,
                    "track should reset after MAX_MISSED_FRAMES");

    if (!passed)
        return 1;

    puts("PASS: Kalman sample sequence");
    return 0;
}
