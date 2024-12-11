#include "shape.h"

Shape shape_create_circle(float radius) {
    return (Shape) {
        .type = CIRCLE_SHAPE,
        .as = (CircleShape) { .radius = radius }
    };
}

Shape shape_create_polygon(Vec2Array vertices) {
    return (Shape) {};
}

Shape shape_create_box(float width, float height) {
    return (Shape) {};
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
