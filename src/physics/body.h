#ifndef BODY_H
#define BODY_H

#include "vec2.h"
#include "shape.h"

typedef struct Body {
    Shape shape;

    // linear motion
    Vec2 position;
    Vec2 velocity;
    Vec2 acceleration;

    // angular motion
    float rotation;
    float angular_velocity;
    float angular_acceleration;

    // forces and torque
    Vec2 sum_forces;
    float sum_torque;

    // mass and moment of inertia
    float inv_mass;
    float inv_I;
} Body;

Body body_create_circle(float radius, int x, int y, float mass);
Body body_create_polygon(Vec2Array vertices, int x, int y, float mass);
Body body_create_box(float width, float height, int x, int y, float mass);
void body_init_circle(Body* body, float circle_radius, int x, int y, float mass);
void body_init_polygon(Body* body, Vec2Array vertices, int x, int y, float mass);
void body_init_box(Body* body, float width, float height, int x, int y, float mass);
void body_integrate_linear(Body* body, float dt);
void body_integrate_angular(Body* body, float dt);
void body_add_force(Body* body, Vec2 force);
void body_add_torque(Body* body, float torque);
void body_clear_forces(Body* body);
void body_clear_torque(Body* body);
void body_update(Body* body, float dt);

#endif //  BODY_H
