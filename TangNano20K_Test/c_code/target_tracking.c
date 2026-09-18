#include "target_tracking.h"

#define KALMAN_DEFAULT_DT        0.1f
#define KALMAN_SIGMA_A_MM_S2  1000.0f
#define KALMAN_R_X_MM2        2500.0f
#define KALMAN_R_Y_MM2        2500.0f

void Tracking_Update(const ld2450_frame_t *frame,
                     KalmanTrack tracks[LD2450_TARGET_COUNT],
                     float dt_s,
                     TrackingResult *result)
{
    const float gate_squared = TRACKING_GATE_MM * TRACKING_GATE_MM;
    const float kalman_dt = (dt_s > 0.0f && dt_s <= TRACKING_MAX_DT_S) ?
                            dt_s : KALMAN_DEFAULT_DT;
    uint8_t detection_used = 0u;
    uint8_t track_used = 0u;
    uint32_t track;
    uint32_t detection;
    uint32_t pair;

    result->matched_mask = 0u;
    result->created_mask = 0u;
    result->reset_mask = 0u;
    for (track = 0; track < LD2450_TARGET_COUNT; track++) {
        result->detection_for_track[track] = -1;
        if (tracks[track].initialized) {
            Kalman_SetDt(&tracks[track], kalman_dt);
            Kalman_Predict(&tracks[track]);
        }
    }

    /* Global nearest pair first; masks enforce one-to-one association. */
    for (pair = 0; pair < LD2450_TARGET_COUNT; pair++) {
        float best_distance = gate_squared;
        int32_t best_track = -1;
        int32_t best_detection = -1;

        for (track = 0; track < LD2450_TARGET_COUNT; track++) {
            if (!tracks[track].initialized || (track_used & (1u << track)))
                continue;
            for (detection = 0; detection < LD2450_TARGET_COUNT; detection++) {
                float dx;
                float dy;
                float distance_squared;

                if (!frame->target[detection].valid ||
                    (detection_used & (1u << detection)))
                    continue;
                dx = (float)frame->target[detection].x_mm - tracks[track].x;
                dy = (float)frame->target[detection].y_mm - tracks[track].y;
                distance_squared = dx * dx + dy * dy;
                if (distance_squared <= best_distance) {
                    best_distance = distance_squared;
                    best_track = (int32_t)track;
                    best_detection = (int32_t)detection;
                }
            }
        }

        if (best_track < 0)
            break;
        track_used |= 1u << (uint32_t)best_track;
        detection_used |= 1u << (uint32_t)best_detection;
        result->detection_for_track[best_track] = (int8_t)best_detection;
        result->matched_mask |= 1u << (uint32_t)best_track;
        Kalman_Update(&tracks[best_track],
                      (float)frame->target[best_detection].x_mm,
                      (float)frame->target[best_detection].y_mm);
        tracks[best_track].active = 1u;
        tracks[best_track].missed_frames = 0u;
    }

    for (track = 0; track < LD2450_TARGET_COUNT; track++) {
        if (!tracks[track].initialized || (track_used & (1u << track)))
            continue;
        if (tracks[track].missed_frames != 0xffu)
            tracks[track].missed_frames++;
        if (tracks[track].missed_frames > TRACKING_MAX_MISSES) {
            Kalman_Reset(&tracks[track]);
            result->reset_mask |= 1u << track;
        }
    }

    for (detection = 0; detection < LD2450_TARGET_COUNT; detection++) {
        if (!frame->target[detection].valid ||
            (detection_used & (1u << detection)))
            continue;
        for (track = 0; track < LD2450_TARGET_COUNT; track++) {
            if (tracks[track].initialized)
                continue;
            Kalman_Init(&tracks[track],
                        (float)frame->target[detection].x_mm,
                        (float)frame->target[detection].y_mm,
                        kalman_dt,
                        KALMAN_SIGMA_A_MM_S2,
                        KALMAN_R_X_MM2,
                        KALMAN_R_Y_MM2);
            result->detection_for_track[track] = (int8_t)detection;
            result->matched_mask |= 1u << track;
            result->created_mask |= 1u << track;
            detection_used |= 1u << detection;
            break;
        }
    }
}
