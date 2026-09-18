#ifndef TARGET_TRACKING_H
#define TARGET_TRACKING_H

#include "std_int_types.h"
#include "ld2450.h"
#include "kalman.h"

#define TRACKING_GATE_MM       1000.0f
#define TRACKING_MAX_MISSES          5u
#define TRACKING_MAX_DT_S          0.25f

typedef struct
{
    int8_t detection_for_track[LD2450_TARGET_COUNT];
    uint8_t matched_mask;
    uint8_t created_mask;
    uint8_t reset_mask;
} TrackingResult;

void Tracking_Update(const ld2450_frame_t *frame,
                     KalmanTrack tracks[LD2450_TARGET_COUNT],
                     float dt_s,
                     TrackingResult *result);

#endif
