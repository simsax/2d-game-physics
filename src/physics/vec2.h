#ifndef VEC2_H
#define VEC2_H

#include <stdint.h>

#define VEC2(x, y) (Vec2) {(x), (y)}
#define vec2_scale vec2_mult

typedef struct {
    float x;
    float y;
} Vec2;

typedef struct {
    uint32_t capacity;
    uint32_t count;
    Vec2* items;
} Vec2Array;

Vec2 vec2_sub(Vec2 a, Vec2 b);
Vec2 vec2_add(Vec2 a, Vec2 b);
Vec2 vec2_mult(Vec2 v, float f);
Vec2 vec2_div(Vec2 v, float f);
Vec2 vec2_rotate(Vec2 v, float angle);
float vec2_magnitude(Vec2 v);
float vec2_magnitude_squared(Vec2 v);
Vec2 vec2_normalize(Vec2 v);
Vec2 vec2_normal(Vec2 v);
float vec2_dot(Vec2 a, Vec2 b);
float vec2_cross(Vec2 a, Vec2 b); // returns magnitude of vector perpendicular to the screen

#endif // VEC2_H
