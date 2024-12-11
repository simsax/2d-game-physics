#include "force.h"
#include "vec2.h"
#include "utils.h"

// TODO: in most functions we both calculate the magnitude and then normalize
// some operations are repeated twice (es vec_magnitude followed by vec_normalize)

Vec2 force_generate_drag(Body* body, float k) {
    Vec2 drag_force = VEC2(0, 0);
    float magnitude_squared = vec_magnitude_squared(body->velocity);
    if (magnitude_squared > 0) {
        float drag_magnitude = k * magnitude_squared;
        Vec2 drag_direction = vec_mult(vec_normalize(body->velocity), -1);
        drag_force = vec_mult(drag_direction, drag_magnitude);
    }
    return drag_force;
}

Vec2 force_generate_friction(Body* body, float k) {
    return vec_mult(vec_normalize(body->velocity), -k);
}

Vec2 force_generate_gravitational(Body* a, Body* b, float G, float min_dist, float max_dist) {
    Vec2 direction = vec_sub(b->position, a->position);
    float direction_magnitude_squared = vec_magnitude_squared(direction);

    // clamp between min_dist and max_dist
    direction_magnitude_squared = clamp(direction_magnitude_squared, min_dist, max_dist);

    Vec2 direction_normalized = vec_normalize(direction);
    return vec_mult(direction_normalized, G / (direction_magnitude_squared * a->inv_mass * b->inv_mass));
}

Vec2 force_generate_spring_anchor(Body* body, Vec2 anchor, float rest_length, float k) {
    Vec2 distance = vec_sub(body->position, anchor);
    float dl = vec_magnitude(distance) - rest_length;
    Vec2 direction = vec_normalize(distance);
    return vec_mult(direction, -dl * k);
}

Vec2 force_generate_spring_body(Body* a, Body* b, float rest_length, float k) {
    Vec2 distance = vec_sub(a->position, b->position);
    float dl = vec_magnitude(distance) - rest_length;
    Vec2 direction = vec_normalize(distance);
    return vec_mult(direction, -dl * k);
}
