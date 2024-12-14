#include "collision.h"
#include "contact.h"
#include "shape.h"
#include "vec2.h"

bool collision_iscolliding(Body* a, Body* b, Contact* contact) {
    bool a_is_circle = a->shape.type == CIRCLE_SHAPE;
    bool b_is_circle = b->shape.type == CIRCLE_SHAPE;
    bool a_is_polygon = a->shape.type == POLYGON_SHAPE || a->shape.type == BOX_SHAPE;
    bool b_is_polygon = b->shape.type == POLYGON_SHAPE || b->shape.type == BOX_SHAPE;

    if (a_is_circle && b_is_circle) {
        return collision_iscolliding_circlecircle(a, b, contact);
    }
    if (a_is_polygon && b_is_polygon) {
        return collision_iscolliding_polygonpolygon(a, b, contact);
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

bool collision_iscolliding_polygonpolygon(Body* a, Body* b, Contact* contact) {
    PolygonShape* a_shape = &a->shape.as.polygon;
    PolygonShape* b_shape = &b->shape.as.polygon;
    Vec2 a_normal, b_normal;
    Vec2 a_point, b_point;
    float ab_separation = shape_polygon_find_min_separation(a_shape, b_shape, &a_normal, &a_point);
    if (ab_separation >= 0)
        return false;
    float ba_separation = shape_polygon_find_min_separation(b_shape, a_shape, &b_normal, &b_point);
    if (ba_separation >= 0)
        return false;

    contact->a = a;
    contact->b = b;

    if (ab_separation > ba_separation) {
        contact->depth = -ab_separation;
        contact->normal = a_normal;
        contact->start = a_point;
        contact->end = vec_add(a_point, vec_mult(contact->normal, contact->depth));
    } else {
        contact->depth = -ba_separation;
        contact->normal = vec_mult(b_normal, -1); // normal always goes from A to B
        contact->start = vec_add(b_point, vec_mult(contact->normal, -contact->depth));
        contact->end = b_point;
    }

    return true;
}
