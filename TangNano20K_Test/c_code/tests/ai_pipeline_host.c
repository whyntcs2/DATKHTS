#include <stdio.h>

#include "../motion_features.h"

int main(void)
{
    TargetFeatureState state;
    AI_Features features;
    float normalized[AI_FEATURE_COUNT];
    int8_t input[AI_FEATURE_COUNT];
    float dt;
    float x;
    float y;

    Feature_Reset(&state);
    while (scanf("%f,%f,%f", &dt, &x, &y) == 3) {
        uint32_t i;

        if (!AI_Pipeline_Push(&state, x, y, dt,
                              &features, normalized, input))
            continue;
        printf("%lu", (unsigned long)(state.finalized_count - AI_WINDOW_FRAMES));
        for (i = 0; i < AI_FEATURE_COUNT; i++)
            printf(",%.9g", (double)features.value[i]);
        for (i = 0; i < AI_FEATURE_COUNT; i++)
            printf(",%.9g", (double)normalized[i]);
        for (i = 0; i < AI_FEATURE_COUNT; i++)
            printf(",%d", (int)input[i]);
        putchar('\n');
    }
    return 0;
}
