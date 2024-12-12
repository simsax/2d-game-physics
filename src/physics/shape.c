#include "shape.h"

Shape shape_create_circle(float radius) {
    return (Shape) {
        .type = CIRCLE_SHAPE,
        .as.circle = (CircleShape) { .radius = radius }
    };
}

Shape shape_create_polygon(Vec2Array vertices) {
    return (Shape) {};
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
            return 0;
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
