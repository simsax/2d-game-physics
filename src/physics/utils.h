#ifndef UTILS_H
#define UTILS_H

#include <math.h>

#define PI 3.14159265

extern const int FPS;
extern const float SECS_PER_FRAME;
extern const int PIXELS_PER_METER;

float clamp(float value, float min, float max);

#endif
