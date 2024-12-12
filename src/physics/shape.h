#ifndef SHAPE_H
#define SHAPE_H

#include "array.h"

typedef enum {
    CIRCLE_SHAPE,
    POLYGON_SHAPE,
    BOX_SHAPE
} ShapeType;

typedef struct {
    int radius;
} CircleShape;

typedef struct {
    Vec2Array local_vertices;
    Vec2Array world_vertices;
} PolygonShape;

typedef struct {
    PolygonShape polygon;
    float width;
    float height;
} BoxShape;

typedef struct {
    ShapeType type;
    union {
        CircleShape circle;
        PolygonShape polygon;
        BoxShape box;
    } as;
} Shape;

Shape shape_create_circle(float radius);
Shape shape_create_polygon(Vec2Array vertices);
Shape shape_create_box(float width, float height);
float shape_moment_of_inertia(Shape* shape);

// rotate and translate shape vertices from "local space" to "world space"
void shape_polygon_update_vertices(PolygonShape* shape, float angle, Vec2 position);

#endif // SHAPE_H
