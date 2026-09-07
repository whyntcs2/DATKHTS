#ifndef LD2450_H
#define LD2450_H

#include "std_int_types.h"

#define LD2450_TARGET_COUNT 3

typedef struct {
    int16_t x_mm;
    int16_t y_mm;
    int16_t speed_cm_s;
    uint16_t resolution_mm;
    uint8_t valid;
} ld2450_target_t;

typedef struct {
    ld2450_target_t target[LD2450_TARGET_COUNT];
} ld2450_frame_t;

int ld2450_configure_multi_target(void);
void ld2450_clear_rx(void);
int ld2450_read_frame(ld2450_frame_t *frame);

#endif
