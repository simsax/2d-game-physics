#include "vec2.h"
#include <math.h>

Vec2 vec2_sub(Vec2 a, Vec2 b) {
    return (Vec2) {a.x - b.x, a.y - b.y};
}

Vec2 vec2_add(Vec2 a, Vec2 b) {
    return (Vec2) {a.x + b.x, a.y + b.y};
}

Vec2 vec2_mult(Vec2 v, float f) {
    return (Vec2) {v.x * f, v.y * f};
}

Vec2 vec2_div(Vec2 v, float f) {
    return (Vec2) {v.x / f, v.y / f};
}

Vec2 vec2_rotate(Vec2 v, float angle) {
    return (Vec2) {
        v.x * cosf(angle) - v.y * sinf(angle),
        v.x * sinf(angle) + v.y * cosf(angle)
    };
}

float vec2_magnitude(Vec2 v) {
    return sqrtf(v.x * v.x + v.y * v.y);
}

float vec2_magnitude_squared(Vec2 v) {
    return v.x * v.x + v.y * v.y;
}

Vec2 vec2_normalize(Vec2 v) {
    float magnitude = vec2_magnitude(v);
    if (magnitude != 0.0f)
        return (Vec2) {.x = v.x / magnitude, .y = v.y / magnitude};
    else
        return v;
}

Vec2 vec2_normal(Vec2 v) {
    return vec2_normalize((Vec2) { v.y, -v.x });
}

float vec2_dot(Vec2 a, Vec2 b) {
    return a.x * b.x + a.y * b.y;
}

float vec2_cross(Vec2 a, Vec2 b) {
    return a.x * b.y - b.x * a.y;
}
