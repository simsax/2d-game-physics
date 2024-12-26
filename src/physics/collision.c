#include "collision.h"
#include "contact.h"
#include "shape.h"
#include "vec2.h"
#include <float.h>

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
    if (a_is_polygon && b_is_circle) {
        return collision_iscolliding_polygoncircle(a, b, contact);
    }
    if (a_is_circle && b_is_polygon) {
        return collision_iscolliding_polygoncircle(b, a, contact);
    }
    return false;
}

bool collision_iscolliding_circlecircle(Body* a, Body* b, Contact* contact) {
    CircleShape* a_shape = &a->shape.as.circle;
    CircleShape* b_shape = &b->shape.as.circle;

    int radius_sum = a_shape->radius + b_shape->radius;
    Vec2 distance = vec2_sub(b->position, a->position);
    bool is_colliding = vec2_magnitude_squared(distance) <= radius_sum * radius_sum;

    if (!is_colliding)
        return false;

    // compute contact collision information
    contact->a = a;
    contact->b = b;
    contact->normal = vec2_normalize(distance);
    contact->start = vec2_add(b->position, vec2_mult(contact->normal, -b_shape->radius));
    contact->end = vec2_add(a->position, vec2_mult(contact->normal, a_shape->radius));
    contact->depth = vec2_magnitude(vec2_sub(contact->end, contact->start));

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
        contact->end = vec2_add(a_point, vec2_mult(contact->normal, contact->depth));
    } else {
        contact->depth = -ba_separation;
        contact->normal = vec2_mult(b_normal, -1); // normal always goes from A to B
        contact->start = vec2_add(b_point, vec2_mult(contact->normal, -contact->depth));
        contact->end = b_point;
    }

    return true;
}

bool collision_iscolliding_polygoncircle(Body* polygon, Body* circle, Contact* contact) {
    // compute the nearest edge
    PolygonShape* polygon_shape = &polygon->shape.as.polygon;
    Vec2Array polygon_vertices = polygon_shape->world_vertices;

    bool inside = true;
    Vec2 min_cur_vertex;
    Vec2 min_next_vertex;
    Vec2 min_normal;
    float distance_circle_edge = -FLT_MAX;
    for (int i = 0; i < polygon_vertices.count; i++) {
        Vec2 va = polygon_vertices.items[i];
        Vec2 edge = shape_polygon_edge_at(polygon_shape, i);
        Vec2 normal = vec2_normal(edge);
        Vec2 va_vc = vec2_sub(circle->position, va);
        float proj = vec2_dot(va_vc, normal);

        // if dot product is in the positive side of the normal, the circle center is outside the polygon
        if (proj > 0 && proj > distance_circle_edge) {
            inside = false;
            distance_circle_edge = proj;
            min_cur_vertex = polygon_vertices.items[i];
            min_next_vertex = polygon_vertices.items[(i + 1) % polygon_vertices.count];
            min_normal = normal;
        } else {
            // circle center is inside, find least negative projection (closest polygon edge)
            if (proj > distance_circle_edge) {
                distance_circle_edge = proj;
                min_cur_vertex = polygon_vertices.items[i];
                min_next_vertex = polygon_vertices.items[(i + 1) % polygon_vertices.count];
                min_normal = normal;
            }
        }
    }

    // compute collision information
    float circle_radius = circle->shape.as.circle.radius;
    if (!inside) {
        // check if circle center is in region A
        Vec2 ac = vec2_sub(circle->position, min_cur_vertex);
        Vec2 perp_normal = vec2_normal(min_normal);

        if (vec2_dot(ac, perp_normal) > 0) {
            Vec2 contact_direction = ac;
            float mag = vec2_magnitude(contact_direction);
            if (mag > circle_radius) {
                // no collision
                return false;
            }
            contact->a = polygon;
            contact->b = circle;
            contact->normal = vec2_normalize(contact_direction);
            contact->start = vec2_add(circle->position, vec2_mult(contact->normal, -circle_radius));
            contact->end = min_cur_vertex;
            contact->depth = circle_radius - mag;
        } else {
            Vec2 bc = vec2_sub(circle->position, min_next_vertex);
            perp_normal = vec2_mult(perp_normal, -1);

            // check if circle in region B
            if (vec2_dot(bc, perp_normal) > 0) {
                Vec2 contact_direction = bc;
                float mag = vec2_magnitude(contact_direction);
                if (mag > circle_radius) {
                    // no collision
                    return false;
                }
                contact->a = polygon;
                contact->b = circle;
                contact->normal = vec2_normalize(contact_direction);
                contact->start = vec2_add(circle->position, vec2_mult(contact->normal, -circle_radius));
                contact->end = min_next_vertex;
                contact->depth = circle_radius - mag;
            } else {
                // circle is in region C, check if colliding
                if (distance_circle_edge > circle_radius) {
                    return false;
                }
                contact->a = polygon;
                contact->b = circle;
                contact->normal = min_normal;
                contact->start = vec2_add(circle->position, vec2_mult(contact->normal, -circle_radius));
                contact->depth = circle_radius - distance_circle_edge;
                contact->end = vec2_add(contact->start, vec2_mult(contact->normal, contact->depth));
            }
        }
    } else {
        // circle center is inside the polygon
        contact->a = polygon;
        contact->b = circle;
        contact->normal = min_normal;
        contact->start = vec2_add(circle->position, vec2_mult(contact->normal, -circle_radius));
        contact->depth = circle_radius - distance_circle_edge;
        contact->end = vec2_add(contact->start, vec2_mult(contact->normal, contact->depth));
    }

    return true;
}
