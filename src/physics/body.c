#include "body.h"
#include "shape.h"
#include "utils.h"
#include "vec2.h"
#include <stdbool.h>
#include <math.h>

void body_init_circle(Body* body, float radius, int x, int y, float mass) {
    shape_init_circle(&body->shape, radius);
    float I = shape_moment_of_inertia(&body->shape) * mass;
    body->position = VEC2(x, y);
    body->velocity = VEC2(0, 0);
    body->acceleration = VEC2(0, 0);
    body->rotation = 0;
    body->angular_velocity = 0;
    body->angular_acceleration = 0;
    body->sum_forces = VEC2(0, 0);
    body->sum_torque = 0;
    body->inv_mass = mass != 0.0f ? 1.0f / mass : 0.0f;
    body->inv_I = I != 0.0f ? 1.0f / I : 0.0f ;
    body->restitution = 1.0f;
    body->friction = 0.7f;
}

void body_init_polygon(Body* body, Vec2Array vertices, int x, int y, float mass) {
    shape_init_polygon(&body->shape, vertices);
    float I = shape_moment_of_inertia(&body->shape) * mass;
    body->position = VEC2(x, y);
    shape_update_vertices(&body->shape, 0, body->position);
    body->velocity = VEC2(0, 0);
    body->acceleration = VEC2(0, 0);
    body->rotation = 0;
    body->angular_velocity = 0;
    body->angular_acceleration = 0;
    body->sum_forces = VEC2(0, 0);
    body->sum_torque = 0;
    body->inv_mass = mass != 0.0f ? 1.0f / mass : 0.0f;
    body->inv_I = I != 0.0f ? 1.0f / I : 0.0f ;
    body->restitution = 1.0f;
    body->friction = 0.7f;
}

void body_init_box(Body* body, float width, float height, int x, int y, float mass) {
    shape_init_box(&body->shape, width, height);
    float I = shape_moment_of_inertia(&body->shape) * mass;
    body->position = VEC2(x, y);
    shape_update_vertices(&body->shape, 0, body->position);
    body->velocity = VEC2(0, 0);
    body->acceleration = VEC2(0, 0);
    body->rotation = 0;
    body->angular_velocity = 0;
    body->angular_acceleration = 0;
    body->sum_forces = VEC2(0, 0);
    body->sum_torque = 0;
    body->inv_mass = mass != 0.0f ? 1.0f / mass : 0.0f;
    body->inv_I = I != 0.0f ? 1.0f / I : 0.0f ;
    body->restitution = 1.0f;
    body->friction = 0.7f;
}

void body_init_circle_pixels(Body* body, int radius, int x, int y, float mass) {
    shape_init_circle(&body->shape, pixels_to_meters(radius));
    float I = shape_moment_of_inertia(&body->shape) * mass;
    body->position = VEC2(pixels_to_meters(x), pixels_to_meters(y));
    body->velocity = VEC2(0, 0);
    body->acceleration = VEC2(0, 0);
    body->rotation = 0;
    body->angular_velocity = 0;
    body->angular_acceleration = 0;
    body->sum_forces = VEC2(0, 0);
    body->sum_torque = 0;
    body->inv_mass = mass != 0.0f ? 1.0f / mass : 0.0f;
    body->inv_I = I != 0.0f ? 1.0f / I : 0.0f ;
    body->restitution = 1.0f;
    body->friction = 0.7f;
}

void body_init_polygon_pixels(Body* body, Vec2Array vertices, int x, int y, float mass) {
    for (uint32_t i = 0; i < vertices.count; i++) {
        Vec2 v = vertices.items[i];
        vertices.items[i] = VEC2(pixels_to_meters(v.x), pixels_to_meters(v.y));
    }
    shape_init_polygon(&body->shape, vertices);
    float I = shape_moment_of_inertia(&body->shape) * mass;
    body->position = VEC2(pixels_to_meters(x), pixels_to_meters(y));
    shape_update_vertices(&body->shape, 0, body->position);
    body->velocity = VEC2(0, 0);
    body->acceleration = VEC2(0, 0);
    body->rotation = 0;
    body->angular_velocity = 0;
    body->angular_acceleration = 0;
    body->sum_forces = VEC2(0, 0);
    body->sum_torque = 0;
    body->inv_mass = mass != 0.0f ? 1.0f / mass : 0.0f;
    body->inv_I = I != 0.0f ? 1.0f / I : 0.0f ;
    body->restitution = 1.0f;
    body->friction = 0.7f;
}

void body_init_box_pixels(Body* body, float width, float height, int x, int y, float mass) {
    shape_init_box(&body->shape, pixels_to_meters(width), pixels_to_meters(height));
    float I = shape_moment_of_inertia(&body->shape) * mass;
    body->position = VEC2(pixels_to_meters(x), pixels_to_meters(y));
    shape_update_vertices(&body->shape, 0, body->position);
    body->velocity = VEC2(0, 0);
    body->acceleration = VEC2(0, 0);
    body->rotation = 0;
    body->angular_velocity = 0;
    body->angular_acceleration = 0;
    body->sum_forces = VEC2(0, 0);
    body->sum_torque = 0;
    body->inv_mass = mass != 0.0f ? 1.0f / mass : 0.0f;
    body->inv_I = I != 0.0f ? 1.0f / I : 0.0f ;
    body->restitution = 1.0f;
    body->friction = 0.7f;
}


void body_add_force(Body* body, Vec2 force) {
    body->sum_forces = vec2_add(body->sum_forces, force);
}

void body_add_torque(Body* body, float torque) {
    body->sum_torque += torque;
}

void body_clear_forces(Body* body) {
    body->sum_forces = VEC2(0, 0);
}

void body_clear_torque(Body* body) {
    body->sum_torque = 0;
}

void body_integrate_forces(Body* body, float dt) {
    if (body_is_static(body))
        return;

    // linear
    body->acceleration = vec2_mult(body->sum_forces, body->inv_mass);
    body->velocity = vec2_add(body->velocity, vec2_scale(body->acceleration, dt));

    // angular
    body->angular_acceleration = body->sum_torque * body->inv_I;
    body->angular_velocity += body->angular_acceleration * dt;
    body->angular_velocity *= 0.99f;

    body_clear_forces(body);
    body_clear_torque(body);
}

void body_integrate_velocities(Body* body, float dt) {
    /*if (body_is_static(body))*/
    /*    return;*/

    // integrate velocities to find new position and rotation
    body->position = vec2_add(body->position, vec2_scale(body->velocity, dt));
    body->rotation += body->angular_velocity * dt;

    // update the vertices according to new position and rotation
    shape_update_vertices(&body->shape, body->rotation, body->position);
}

bool body_is_static(Body* body) {
    float epsilon = 1e-8f;
    return (float) fabs(body->inv_mass - 0.0f) < epsilon;
}

void body_apply_impulse_at_point(Body* body, Vec2 jn, Vec2 r) {
    if (body_is_static(body))
        return;
    body->velocity = vec2_add(body->velocity, vec2_mult(jn, body->inv_mass));
    body->angular_velocity += vec2_cross(r, jn) * body->inv_I;
}

void body_apply_impulse_linear(Body* body, Vec2 jn) {
    if (body_is_static(body))
        return;
    body->velocity = vec2_add(body->velocity, vec2_mult(jn, body->inv_mass));
}

void body_apply_impulse_angular(Body *body, float j) {
    if (body_is_static(body))
        return;
    body->angular_velocity += j * body->inv_I;
}

Vec2 body_local_to_world_space(Body* body, Vec2 point) {
    Vec2 rotated_point = vec2_rotate(point, body->rotation);
    Vec2 translated_point = vec2_add(rotated_point, body->position);
    return translated_point;
}

Vec2 body_world_to_local_space(Body* body, Vec2 point) {
    Vec2 translated_point = vec2_sub(point, body->position);
    Vec2 rotated_point = vec2_rotate(translated_point, -body->rotation);
    return rotated_point;
}

