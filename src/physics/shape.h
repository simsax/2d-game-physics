#ifndef SHAPE_H
#define SHAPE_H

#include "vec2.h"

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

void shape_init_circle(Shape* shape, float radius);
void shape_init_polygon(Shape* shape, Vec2Array local_vertices);
void shape_init_box(Shape* shape, float width, float height);
float shape_moment_of_inertia(Shape* shape);

// rotate and translate shape vertices from "local space" to "world space"
void shape_update_vertices(Shape* shape, float angle, Vec2 position);

// Find edge at a certain vertex index.
// Ex. triangle with vertices A, B, C
// index = 0 -> Edge AB
// index = 1 -> Edge BC
// index = 2 -> Edge CA
Vec2 shape_polygon_edge_at(PolygonShape* shape, int index);
float shape_polygon_find_min_separation(PolygonShape* a, PolygonShape* b, int* index_reference_edge);
int shape_polygon_find_incident_edge_index(PolygonShape* reference, Vec2 normal);
int shape_polygon_clip_segment_to_line(Vec2* contacts_in, Vec2* contacts_out, Vec2 c0, Vec2 c1);

#endif // SHAPE_H
