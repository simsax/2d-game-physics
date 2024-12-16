#include "body.h"
#include "array.h"
#include "shape.h"
#include "vec2.h"
#include <stdbool.h>
#include <math.h>

Body body_create_circle(float radius, int x, int y, float mass) {
    Shape shape = shape_create_circle(radius);
    float I = shape_moment_of_inertia(&shape) * mass;
    return (Body) {
        .shape = shape,
        .position = VEC2(x, y),
        .inv_mass = mass != 0 ? 1.0 / mass : 0,
        .inv_I = I != 0 ? 1.0 / I : 0 ,
        .restitution = 1.0f,
        .friction = 0.7f
    };
}

Body body_create_polygon(Vec2Array vertices, int x, int y, float mass) {
    Shape shape = shape_create_polygon(vertices);
    float I = shape_moment_of_inertia(&shape) * mass;
    return (Body) {
        .shape = shape,
        .position = VEC2(x, y),
        .inv_mass = mass != 0 ? 1.0 / mass : 0,
        .inv_I = I != 0 ? 1.0 / I : 0,
        .restitution = 1.0f,
        .friction = 0.7f
    };
}

Body body_create_box(float width, float height, int x, int y, float mass) {
    Shape shape = shape_create_box(width, height);
    float I = shape_moment_of_inertia(&shape) * mass;
    return (Body) {
        .shape = shape,
        .position = VEC2(x, y),
        .inv_mass = mass != 0 ? 1.0 / mass : 0,
        .inv_I = I != 0 ? 1.0 / I : 0,
        .restitution = 1.0f,
        .friction = 0.7f
    };
}

void body_init_circle(Body* body, float radius, int x, int y, float mass) {
    Shape shape = shape_create_circle(radius);
    float I = shape_moment_of_inertia(&shape) * mass;
    body->shape = shape;
    body->position = (Vec2) { .x = x, .y = y };
    body->inv_mass = mass != 0 ? 1.0 / mass : 0;
    body->inv_I = I != 0 ? 1.0 / I : 0;
    body->velocity = VEC2(0, 0);
    body->acceleration = VEC2(0, 0);
    body->rotation = 0;
    body->angular_velocity = 0;
    body->angular_acceleration = 0;
    body->sum_forces = VEC2(0, 0);
    body->sum_torque = 0;
    body->restitution = 1.0f;
    body->friction = 0.7f;
}

void body_init_polygon(Body* body, Vec2Array vertices, int x, int y, float mass) {
    Shape shape = shape_create_polygon(vertices);
    float I = shape_moment_of_inertia(&shape) * mass;
    body->shape = shape;
    body->position = (Vec2) { .x = x, .y = y };
    body->inv_mass = mass != 0 ? 1.0 / mass : 0;
    body->inv_I = I != 0 ? 1.0 / I : 0;
    body->velocity = VEC2(0, 0);
    body->acceleration = VEC2(0, 0);
    body->rotation = 0;
    body->angular_velocity = 0;
    body->angular_acceleration = 0;
    body->sum_forces = VEC2(0, 0);
    body->sum_torque = 0;
    body->restitution = 1.0f;
    body->friction = 0.7f;
}

void body_init_box(Body* body, float width, float height, int x, int y, float mass) {
    Shape shape = shape_create_box(width, height);
    float I = shape_moment_of_inertia(&shape) * mass;
    body->shape = shape;
    body->position = (Vec2) { .x = x, .y = y };
    body->inv_mass = mass != 0 ? 1.0 / mass : 0;
    body->inv_I = I != 0 ? 1.0 / I : 0;
    body->velocity = VEC2(0, 0);
    body->acceleration = VEC2(0, 0);
    body->rotation = 0;
    body->angular_velocity = 0;
    body->angular_acceleration = 0;
    body->sum_forces = VEC2(0, 0);
    body->sum_torque = 0;
    body->restitution = 1.0f;
    body->friction = 0.7f;
}

void body_integrate_linear(Body* body, float dt) {
    if (body_is_static(body))
        return;

    // find acceleration based on the forces being applied and the mass
    body->acceleration = vec_mult(body->sum_forces, body->inv_mass);

    // integrate acceleration to find new velocity
    body->velocity = vec_add(body->velocity, vec_scale(body->acceleration, dt));

    // integrate velocity to find new position
    body->position = vec_add(body->position, vec_scale(body->velocity, dt));

    body_clear_forces(body);
}

void body_integrate_angular(Body* body, float dt) {
    if (body_is_static(body))
        return;

    body->angular_acceleration = body->sum_torque * body->inv_I;
    body->angular_velocity += body->angular_acceleration * dt;
    body->rotation += body->angular_velocity * dt;
    body_clear_torque(body);
}


void body_add_force(Body* body, Vec2 force) {
    body->sum_forces = vec_add(body->sum_forces, force);
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

void body_update(Body* body, float dt) {
    // integrate forces to find new position and rotation
    body_integrate_linear(body, dt);
    body_integrate_angular(body, dt);

    // rotate and translate body vertices from local space to world space
    bool is_polygon = body->shape.type == POLYGON_SHAPE || body->shape.type == BOX_SHAPE;
    if (is_polygon) {
        PolygonShape* polygon_shape = &body->shape.as.polygon;
        shape_polygon_update_vertices(polygon_shape, body->rotation, body->position);
    }
}

bool body_is_static(Body* body) {
    float epsilon = 1e-8;
    return fabs(body->inv_mass - 0.0f) < epsilon;
}

void body_apply_impulse(Body* body, Vec2 jn, Vec2 r) {
    if (body_is_static(body))
        return;

    body->velocity = vec_add(body->velocity, vec_mult(jn, body->inv_mass));
    body->angular_velocity += vec_cross(r, jn) * body->inv_I;
}

void body_apply_impulse_linear(Body* body, Vec2 jn) {
    if (body_is_static(body))
        return;

    body->velocity = vec_add(body->velocity, vec_mult(jn, body->inv_mass));
}

void body_apply_impulse_angular(Body *body, Vec2 jn, Vec2 r) {
    if (body_is_static(body))
        return;

    body->angular_velocity += vec_cross(r, jn) * body->inv_I;;
}
