#include "collision.h"
#include "shape.h"
#include "vec2.h"
#include <float.h>
#include <string.h>

bool collision_iscolliding(Body* a, Body* b, Contact* contacts, uint32_t* num_contacts) {
    bool a_is_circle = a->shape.type == CIRCLE_SHAPE;
    bool b_is_circle = b->shape.type == CIRCLE_SHAPE;
    bool a_is_polygon = a->shape.type == POLYGON_SHAPE || a->shape.type == BOX_SHAPE;
    bool b_is_polygon = b->shape.type == POLYGON_SHAPE || b->shape.type == BOX_SHAPE;

    if (a_is_circle && b_is_circle) {
        return collision_iscolliding_circlecircle(a, b, contacts, num_contacts);
    }
    if (a_is_polygon && b_is_polygon) {
        return collision_iscolliding_polygonpolygon(a, b, contacts, num_contacts);
    }
    if (a_is_polygon && b_is_circle) {
        return collision_iscolliding_polygoncircle(a, b, contacts, num_contacts);
    }
    if (a_is_circle && b_is_polygon) {
        return collision_iscolliding_polygoncircle(b, a, contacts, num_contacts);
    }
    return false;
}

bool collision_iscolliding_circlecircle(Body* a, Body* b, Contact* contacts, uint32_t* num_contacts) {
    *num_contacts = 1;
    CircleShape* a_shape = &a->shape.as.circle;
    CircleShape* b_shape = &b->shape.as.circle;

    float radius_sum = a_shape->radius + b_shape->radius;
    Vec2 distance = vec2_sub(b->position, a->position);
    bool is_colliding = vec2_magnitude_squared(distance) <= radius_sum * radius_sum;

    if (!is_colliding)
        return false;

    Contact* contact = &contacts[0];

    // compute contact collision information
    contact->normal = vec2_normalize(distance);
    contact->start = vec2_add(b->position, vec2_mult(contact->normal, -b_shape->radius));
    contact->end = vec2_add(a->position, vec2_mult(contact->normal, a_shape->radius));
    contact->depth = vec2_magnitude(vec2_sub(contact->end, contact->start));

    return true;
}

bool collision_iscolliding_polygonpolygon(Body* a, Body* b, Contact* contacts, uint32_t* num_contacts) {
    PolygonShape* a_shape = &a->shape.as.polygon;
    PolygonShape* b_shape = &b->shape.as.polygon;
    int a_index_reference_edge, b_index_reference_edge;
    float ab_separation = shape_polygon_find_min_separation(a_shape, b_shape, &a_index_reference_edge);
    if (ab_separation >= 0)
        return false;
    float ba_separation = shape_polygon_find_min_separation(b_shape, a_shape, &b_index_reference_edge);
    if (ba_separation >= 0)
        return false;

    PolygonShape* reference_shape;
    PolygonShape* incident_shape;
    uint32_t index_reference_edge;

    if (ab_separation > ba_separation) {
        reference_shape = a_shape;
        incident_shape = b_shape;
        index_reference_edge = a_index_reference_edge;
    } else {
        reference_shape = b_shape;
        incident_shape = a_shape;
        index_reference_edge = b_index_reference_edge;
    }

    // find reference edge based on index returned from the function
    Vec2 reference_edge = shape_polygon_edge_at(reference_shape, index_reference_edge);

    // clipping
    int incident_index = shape_polygon_find_incident_edge_index(incident_shape, vec2_normal(reference_edge));
    int incident_next_index = (incident_index + 1) % incident_shape->world_vertices.count;
    Vec2 v0 = incident_shape->world_vertices.items[incident_index];
    Vec2 v1 = incident_shape->world_vertices.items[incident_next_index];

    Vec2 contact_points[2] = { v0, v1 };
    Vec2 clipped_points[2] = { v0, v1 };
    // TODO: figure out for loop
    for (uint32_t i = 0; i < reference_shape->world_vertices.count; i++) {
        if (i == index_reference_edge)
            continue;
        Vec2 c0 = reference_shape->world_vertices.items[i];
        Vec2 c1 = reference_shape->world_vertices.items[(i + 1) % reference_shape->world_vertices.count];
        int num_clipped = shape_polygon_clip_segment_to_line(contact_points, clipped_points, c0, c1);
        if (num_clipped < 2)
            break;
        // make the next contact points the ones that were just clipped
        memcpy(contact_points, clipped_points, sizeof(contact_points)); 
    }

    Vec2 v_ref = reference_shape->world_vertices.items[index_reference_edge];
    // consider only clipped points whose separation is negative (objects are penetrating)
    for (int i = 0; i < 2; i++) {
        Vec2 v_clip = clipped_points[i];
        Vec2 ref_normal = vec2_normal(reference_edge);
        float separation = vec2_dot(vec2_sub(v_clip, v_ref), ref_normal);
        if (separation <= 0) {
            Contact* contact = &contacts[(*num_contacts)++];
            contact->normal = ref_normal;
            contact->start = v_clip;
            contact->end = vec2_add(v_clip, vec2_mult(ref_normal, -separation));
            if (ba_separation >= ab_separation) {
                // start, end and normal always from A to B
                // swap start and end
                Vec2 temp = contact->start;
                contact->start = contact->end;
                contact->end = temp;
                
                contact->normal = vec2_mult(contact->normal, -1);
            }
        }
    }

    return true;
}

bool collision_iscolliding_polygoncircle(Body* polygon, Body* circle, Contact* contacts, uint32_t* num_contacts) {
    // compute the nearest edge
    PolygonShape* polygon_shape = &polygon->shape.as.polygon;
    Vec2Array polygon_vertices = polygon_shape->world_vertices;
    *num_contacts = 1;

    bool inside = true;
    Vec2 min_cur_vertex;
    Vec2 min_next_vertex;
    Vec2 min_normal;
    float distance_circle_edge = -FLT_MAX;
    for (uint32_t i = 0; i < polygon_vertices.count; i++) {
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

    Contact* contact = &contacts[0];
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
                contact->normal = vec2_normalize(contact_direction);
                contact->start = vec2_add(circle->position, vec2_mult(contact->normal, -circle_radius));
                contact->end = min_next_vertex;
                contact->depth = circle_radius - mag;
            } else {
                // circle is in region C, check if colliding
                if (distance_circle_edge > circle_radius) {
                    return false;
                }
                contact->normal = min_normal;
                contact->start = vec2_add(circle->position, vec2_mult(contact->normal, -circle_radius));
                contact->depth = circle_radius - distance_circle_edge;
                contact->end = vec2_add(contact->start, vec2_mult(contact->normal, contact->depth));
            }
        }
    } else {
        // circle center is inside the polygon
        contact->normal = min_normal;
        contact->start = vec2_add(circle->position, vec2_mult(contact->normal, -circle_radius));
        contact->depth = circle_radius - distance_circle_edge;
        contact->end = vec2_add(contact->start, vec2_mult(contact->normal, contact->depth));
    }

    return true;
}
