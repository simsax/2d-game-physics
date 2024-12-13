#include "collision.h"
#include "contact.h"
#include "shape.h"
#include "vec2.h"

bool collision_iscolliding(Body* a, Body* b, Contact* contact) {
    bool a_is_circle = a->shape.type == CIRCLE_SHAPE;
    bool b_is_circle = b->shape.type == CIRCLE_SHAPE;

    if (a_is_circle && b_is_circle) {
        return collision_iscolliding_circlecircle(a, b, contact);
    }
    return false;
}

bool collision_iscolliding_circlecircle(Body* a, Body* b, Contact* contact) {
    CircleShape* a_shape = &a->shape.as.circle;
    CircleShape* b_shape = &b->shape.as.circle;

    int radius_sum = a_shape->radius + b_shape->radius;
    Vec2 distance = vec_sub(b->position, a->position);
    bool is_colliding = vec_magnitude_squared(distance) <= radius_sum * radius_sum;

    if (!is_colliding)
        return false;

    // compute contact collision information
    contact->a = a;
    contact->b = b;
    contact->normal = vec_normalize(distance);
    contact->start = vec_add(b->position, vec_mult(contact->normal, -b_shape->radius));
    contact->end = vec_add(a->position, vec_mult(contact->normal, a_shape->radius));
    contact->depth = vec_magnitude(vec_sub(contact->end, contact->start));

    return true;
}
