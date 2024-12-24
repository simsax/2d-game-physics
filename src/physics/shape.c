#include "shape.h"
#include <float.h>
#include "array.h"

Shape shape_create_circle(float radius) {
    return (Shape) {
        .type = CIRCLE_SHAPE,
        .as.circle = (CircleShape) { .radius = radius }
    };
}

Shape shape_create_polygon(Vec2Array vertices) {
    Vec2Array local_vertices = DA_NULL;
    Vec2Array world_vertices = DA_NULL;

    for (int i = 0; i < vertices.count; i++) {
        DA_APPEND(&local_vertices, vertices.items[i]);
        DA_APPEND(&world_vertices, vertices.items[i]);
    }

    return (Shape) {
        .type = POLYGON_SHAPE,
        .as.polygon = (PolygonShape) {
            .local_vertices = local_vertices,
            .world_vertices = world_vertices,
        }
    };
}

Shape shape_create_box(float width, float height) {
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

    return (Shape) {
        .type = BOX_SHAPE,
        .as.box = (BoxShape) {
            .polygon = (PolygonShape) { 
                .local_vertices = local_vertices,
                .world_vertices = world_vertices,
            },
            .width = width,
            .height = height
        }
    };
}

float shape_moment_of_inertia(Shape* shape) {
    // these still need to be multiplied by the mass (done in body.c)
    switch (shape->type) {
        case CIRCLE_SHAPE: {
            float r = shape->as.circle.radius;
            return 0.5 * r * r;
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

void shape_polygon_update_vertices(PolygonShape* shape, float angle, Vec2 position) {
    // loop over all vertices and transform from local to world space
    for (int i = 0; i < shape->local_vertices.count; i++) {
        // first rotate, then translate
        shape->world_vertices.items[i] = vec_rotate(shape->local_vertices.items[i], angle);
        shape->world_vertices.items[i] = vec_add(shape->world_vertices.items[i], position);
    }
}

Vec2 shape_polygon_edge_at(PolygonShape* shape, int index) {
    int num_vertices = shape->world_vertices.count;
    return vec_sub(
                shape->world_vertices.items[(index + 1) % num_vertices],
                shape->world_vertices.items[index]
            );
}

float shape_polygon_find_min_separation(PolygonShape* a, PolygonShape* b, Vec2* axis_normal, Vec2* point) {
    float separation = -FLT_MAX; // -inf

    for (int i = 0; i < a->world_vertices.count; i++) {
        Vec2 va = a->world_vertices.items[i];
        Vec2 edge = shape_polygon_edge_at(a, i);
        Vec2 normal = vec_normal(edge);

        float min_separation = FLT_MAX;
        Vec2 min_vertex;
        for (int j = 0; j < b->world_vertices.count; j++) {
            Vec2 vb = b->world_vertices.items[j];
            Vec2 va_vb = vec_sub(vb, va); // vector from va to vb
            float proj = vec_dot(va_vb, normal);
            if (proj < min_separation) {
                min_separation = proj;
                min_vertex = vb;
            }
        }
        if (min_separation > separation) {
            separation = min_separation;
            *axis_normal = normal;
            *point = min_vertex;
        }

        if (separation > 0) {
            // there is no collision, so no need to keep looping to find the "best" separation
            return separation;
        }
    }

    return separation;
}

