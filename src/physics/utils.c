#include "utils.h"

const int FPS = 60;
const float SECS_PER_FRAME = 1.0 / FPS;
const int PIXELS_PER_METER = 50; // approx 21 meters high

float clamp(float value, float min, float max) {
   return fmax(fmin(max, value), min); 
}
