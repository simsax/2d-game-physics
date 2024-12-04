#include "force.h"
#include "vec2.h"
#include <math.h>

Vec2 force_generate_drag(Particle* particle, float k) {
    Vec2 drag_force = VEC2(0, 0);
    float magnitude_squared = vec_magnitude_squared(particle->velocity);
    if (magnitude_squared > 0) {
        float drag_magnitude = k * magnitude_squared;
        Vec2 drag_direction = vec_mult(vec_normalize(particle->velocity), -1);
        drag_force = vec_mult(drag_direction, drag_magnitude);
    }
    return drag_force;
}

Vec2 force_generate_friction(Particle* particle, float k) {
    return vec_mult(vec_normalize(particle->velocity), -k);
}

Vec2 force_generate_gravitational(Particle* a, Particle* b, float G, float min_dist, float max_dist) {
    Vec2 direction = vec_sub(b->position, a->position);
    float direction_magnitude_squared = vec_magnitude_squared(direction);

    // clamp between min_dist and max_dist
    direction_magnitude_squared = fmax(fmin(max_dist, direction_magnitude_squared), min_dist);

    Vec2 direction_normalized = vec_normalize(direction);
    return vec_mult(direction_normalized, a->mass * b->mass * G / direction_magnitude_squared);
}
