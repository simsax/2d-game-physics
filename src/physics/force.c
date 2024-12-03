#include "force.h"
#include "vec2.h"

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
