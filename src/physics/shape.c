#include "shape.h"
#include "array.h"
#include <float.h>
#include <stdbool.h>


void shape_init_circle(Shape* shape, float radius) {
    shape->type = CIRCLE_SHAPE;
    shape->as.circle = (CircleShape) { .radius = radius };
}

void shape_init_polygon(Shape* shape, Vec2Array local_vertices) {
    Vec2Array world_vertices = DA_NULL;

    for (uint32_t i = 0; i < local_vertices.count; i++) {
        DA_APPEND(&world_vertices, local_vertices.items[i]);
    }

    shape->type = POLYGON_SHAPE;
    shape->as.polygon = (PolygonShape) {
        .local_vertices = local_vertices,
        .world_vertices = world_vertices,
    };
}

void shape_init_box(Shape* shape, float width, float height) {
    float half_width = width / 2.0f;
    float half_height = height / 2.0f;

    // create vertices in local space (wrt the origin)
    // note: implemented like this is more of a local screen space because y-axis points down
    Vec2Array local_vertices = DA_NULL;
    DA_APPEND(&local_vertices, VEC2(-half_width, -half_height));
    DA_APPEND(&local_vertices, VEC2(half_width, -half_height));
    DA_APPEND(&local_vertices, VEC2(half_width, half_height));
    DA_APPEND(&local_vertices, VEC2(-half_width, half_height));

    Vec2Array world_vertices = DA_NULL;
    DA_APPEND(&world_vertices, VEC2(-half_width, -half_height));
    DA_APPEND(&world_vertices, VEC2(half_width, -half_height));
    DA_APPEND(&world_vertices, VEC2(half_width, half_height));
    DA_APPEND(&world_vertices, VEC2(-half_width, half_height));

    shape->type = BOX_SHAPE;
    shape->as.box = (BoxShape) {
        .polygon = (PolygonShape) { 
            .local_vertices = local_vertices,
            .world_vertices = world_vertices,
        },
        .width = width,
        .height = height
    };
}

float shape_moment_of_inertia(Shape* shape) {
    // these still need to be multiplied by the mass (done in body.c)
    switch (shape->type) {
        case CIRCLE_SHAPE: {
            float r = shape->as.circle.radius;
            return 0.5f * r * r;
        } break;
        case POLYGON_SHAPE: {
            // TODO
            return 5000;
        } break;
        case BOX_SHAPE: {
            float w = shape->as.box.width;
            float h = shape->as.box.height;
            // 1/12 * (w^2 + h^2)
            return 0.083333f * (w * w + h * h);
        } break;
    }
    // should never reach this
    return 0;
}

void shape_update_vertices(Shape* shape, float angle, Vec2 position) {
    bool is_circle = shape->type == CIRCLE_SHAPE;
    if (is_circle)
        return;
    PolygonShape* polygon_shape = &shape->as.polygon;
    // loop over all vertices and transform from local to world space
    for (uint32_t i = 0; i < polygon_shape->local_vertices.count; i++) {
        // first rotate, then translate
        polygon_shape->world_vertices.items[i] = vec2_rotate(polygon_shape->local_vertices.items[i], angle);
        polygon_shape->world_vertices.items[i] = vec2_add(polygon_shape->world_vertices.items[i], position);
    }
}

Vec2 shape_polygon_edge_at(PolygonShape* shape, int index) {
    int num_vertices = shape->world_vertices.count;
    return vec2_sub(
                shape->world_vertices.items[(index + 1) % num_vertices],
                shape->world_vertices.items[index]
            );
}

float shape_polygon_find_min_separation(PolygonShape* a, PolygonShape* b, int* index_reference_edge) {
    float separation = -FLT_MAX; // -inf

    for (uint32_t i = 0; i < a->world_vertices.count; i++) {
        Vec2 va = a->world_vertices.items[i];
        Vec2 edge = shape_polygon_edge_at(a, i);
        Vec2 normal = vec2_normal(edge);

        float min_separation = FLT_MAX;
        /*Vec2 min_vertex;*/
        for (uint32_t j = 0; j < b->world_vertices.count; j++) {
            Vec2 vb = b->world_vertices.items[j];
            Vec2 va_vb = vec2_sub(vb, va); // vector from va to vb
            float proj = vec2_dot(va_vb, normal);
            if (proj < min_separation) {
                min_separation = proj;
                /*min_vertex = vb;*/
            }
        }
        if (min_separation > separation) {
            separation = min_separation;
            *index_reference_edge = i;
        }

        if (separation > 0) {
            // there is no collision, so no need to keep looping to find the "best" separation
            return separation;
        }
    }

    return separation;
}

int shape_polygon_find_incident_edge_index(PolygonShape* reference, Vec2 normal) {
    float min_proj = FLT_MAX;
    int incident_edge = -1;
    for (uint32_t i = 0; i < reference->world_vertices.count; i++) {
        Vec2 edge = shape_polygon_edge_at(reference, i);
        Vec2 edge_normal = vec2_normal(edge);

        float proj = vec2_dot(edge_normal, normal);
        if (proj < min_proj) {
            min_proj = proj;
            incident_edge = i;
        }
    }
    return incident_edge;
}

int shape_polygon_clip_segment_to_line(Vec2* contacts_in, Vec2* contacts_out, Vec2 c0, Vec2 c1) {
    int num_out = 0;

    /*Vec2 normal = vec2_normal(vec2_sub(c1, c0));*/
    /*float dist0 = vec2_dot(vec2_sub(contacts_in[0], c0), normal);*/
    /*float dist1 = vec2_dot(vec2_sub(contacts_in[1], c0), normal);*/

    Vec2 normal = vec2_normalize(vec2_sub(c1, c0));
    float dist0 = vec2_cross(vec2_sub(contacts_in[0], c0), normal);
    float dist1 = vec2_cross(vec2_sub(contacts_in[1], c0), normal);

    // if points are behind the plane
    if (dist0 <= 0)
        contacts_out[num_out++] = contacts_in[0];
    if (dist1 <= 0)
        contacts_out[num_out++] = contacts_in[1];

    // if points are on different sides of the plane, clip to plane intersection
    if (dist0 * dist1 < 0) {
        float total_dist = dist0 - dist1;

        // find intersection with linear interpolation: lerp(a, b, t) => a + t * (b - a)
        float t = dist0 / total_dist;
        Vec2 contact = vec2_add(contacts_in[0], vec2_mult(vec2_sub(contacts_in[1], contacts_in[0]), t));
        contacts_out[num_out++] = contact;
    }
    return num_out;
}

