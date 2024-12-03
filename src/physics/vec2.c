#include "vec2.h"
#include <math.h>

Vec2 vec_sub(Vec2 a, Vec2 b) {
    return (Vec2) {a.x - b.x, a.y - b.y};
}

Vec2 vec_add(Vec2 a, Vec2 b) {
    return (Vec2) {a.x + b.x, a.y + b.y};
}

Vec2 vec_mult(Vec2 v, float f) {
    return (Vec2) {v.x * f, v.y * f};
}

Vec2 vec_div(Vec2 v, float f) {
    return (Vec2) {v.x / f, v.y / f};
}

Vec2 vec_rotate(Vec2 v, float angle) {
    return (Vec2) {
        v.x * cos(angle) - v.y * sin(angle),
        v.x * sin(angle) + v.y * cos(angle)
    };
}

float vec_magnitude(Vec2 v) {
    return sqrtf(v.x * v.x + v.y * v.y);
}

float vec_magnitude_squared(Vec2 v) {
    return v.x * v.x + v.y * v.y;
}

Vec2 vec_normalize(Vec2 v) {
    float magnitude = vec_magnitude(v);
    if (magnitude != 0.0f)
        return (Vec2) {.x = v.x / magnitude, .y = v.y / magnitude};
    else
        return v;
}

Vec2 vec_normal(Vec2 v) {
    return vec_normalize((Vec2) { v.y, -v.x });
}

float dot(Vec2 a, Vec2 b) {
    return a.x * b.x + a.y * b.y;
}

float cross(Vec2 a, Vec2 b) {
    return a.x * b.y - b.x * a.y;
}
