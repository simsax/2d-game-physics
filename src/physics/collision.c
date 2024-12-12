#include "collision.h"
#include "shape.h"
#include "vec2.h"

bool collision_iscolliding(Body* a, Body* b) {
    bool a_is_circle = a->shape.type == CIRCLE_SHAPE;
    bool b_is_circle = b->shape.type == CIRCLE_SHAPE;

    if (a_is_circle && b_is_circle) {
        return collision_iscolliding_circlecircle(a, b);
    }
    return false;
}

bool collision_iscolliding_circlecircle(Body* a, Body* b) {
    CircleShape* a_shape = &a->shape.as.circle;
    CircleShape* b_shape = &b->shape.as.circle;

    int radius_sum = a_shape->radius + b_shape->radius;
    Vec2 distance = vec_sub(b->position, a->position);
    bool is_colliding = vec_magnitude_squared(distance) <= radius_sum * radius_sum;
    return is_colliding;
}
