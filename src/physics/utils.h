#ifndef UTILS_H
#define UTILS_H

#include <math.h>

#define FPS 60.0f
#define SECS_PER_FRAME (1.0f / FPS)
#define PIXELS_PER_METER 100.0f // 10.8 meters high for 1080p
#define METERS_PER_PIXEL (1.0f / 100.0f)

static inline float clamp(float value, float min, float max) {
    return fmax(fmin(max, value), min); 
}

static inline float pixels_to_meters(float pixels) {
    return pixels * METERS_PER_PIXEL;
}

static inline float meters_to_pixels(float meters) {
    return meters * PIXELS_PER_METER;
}

#endif
