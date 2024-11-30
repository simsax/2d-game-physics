#ifndef VEC2_H
#define VEC2_H

typedef struct {
    float x;
    float y;
} Vec2;

Vec2 vec2(float x);
Vec2 vec_sub(Vec2 a, Vec2 b);
Vec2 vec_add(Vec2 a, Vec2 b);
Vec2 vec_scale(Vec2 v, float f);
Vec2 vec_rotate(Vec2 v, float angle);
float vec_magnitude(Vec2 v);
float vec_magnitude_squared(Vec2 v);
Vec2 vec_normalize(Vec2 v);
Vec2 vec_normal(Vec2 v);
float dot(Vec2 a, Vec2 b);
float cross(Vec2 a, Vec2 b); // returns magnitude of vector perpendicular to the screen

#endif // VEC2_H
