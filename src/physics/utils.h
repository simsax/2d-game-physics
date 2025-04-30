#ifndef UTILS_H
#define UTILS_H

#include <math.h>

#define MIN_FPS 30.0f
#define MIN_SECS_PER_FRAME (1.0f / MIN_FPS)
#define FIXED_DT 0.008f

extern float PIXELS_PER_METER; 

static inline float clamp(float value, float min, float max) {
    return fmax(fmin(max, value), min); 
}

static inline float pixels_to_meters(float pixels) {
    return pixels / PIXELS_PER_METER;
}

static inline float meters_to_pixels(float meters) {
    return meters * PIXELS_PER_METER;
}

#endif
 
