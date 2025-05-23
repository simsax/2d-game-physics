#ifndef BODY_H
#define BODY_H

#include "vec2.h"
#include "shape.h"
#include <stdbool.h>

typedef struct Body {
    Shape shape;

    // linear motion
    Vec2 position;
    Vec2 prev_position;
    Vec2 velocity;
    Vec2 acceleration;

    // angular motion
    float rotation;
    float angular_velocity;
    float angular_acceleration;

    // forces and torque
    Vec2 sum_forces;
    float sum_torque;
    float static_torque; // hack for demo 4

    // mass and moment of inertia
    float inv_mass;
    float inv_I;

    // coefficients of restitution and friction
    float restitution;
    float friction;
} Body;

typedef struct {
    uint32_t capacity;
    uint32_t count;
    Body* items;
} BodyArray;

void body_init_circle(Body* body, float radius, float x, float y, float mass);
void body_init_polygon(Body* body, Vec2Array vertices, float x, float y, float mass);
void body_init_box(Body* body, float width, float height, float x, float y, float mass);
void body_init_circle_pixels(Body* body, int radius, int x, int y, float mass);
void body_init_circle_container_pixels(Body* body, int radius, int x, int y, float mass);
void body_init_polygon_pixels(Body* body, Vec2Array vertices, int x, int y, float mass);
void body_init_box_pixels(Body* body, float width, float height, int x, int y, float mass);
void body_integrate_linear(Body* body, float dt);
void body_integrate_angular(Body* body, float dt);
void body_add_force(Body* body, Vec2 force);
void body_add_torque(Body* body, float torque);
void body_add_static_torque(Body* body, float torque);
void body_clear_forces(Body* body);
void body_clear_torque(Body* body);
bool body_is_static(Body* body);
void body_apply_impulse_at_point(Body* body, Vec2 jn, Vec2 r);
void body_apply_impulse_linear(Body* body, Vec2 jn);
void body_apply_impulse_angular(Body* body, float j);
Vec2 body_local_to_world_space(Body* body, Vec2 point);
Vec2 body_world_to_local_space(Body* body, Vec2 point);
void body_integrate_forces(Body* body, float dt);
void body_integrate_velocities(Body* body, float dt);

#endif //  BODY_H
