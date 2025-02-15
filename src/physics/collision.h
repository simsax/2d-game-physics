#ifndef COLLISION_H
#define COLLISION_H

#include <stdbool.h>
#include "body.h"

typedef struct {
    int a_index;
    int b_index;

    Vec2 start;
    Vec2 end;

    Vec2 normal;
    float depth;
} Contact;

bool collision_iscolliding(Body* a, Body* b, int a_index, int b_index, Contact* contacts, uint32_t* num_contacts);
bool collision_iscolliding_circlecircle(Body* a, Body* b, int a_index, int b_index, Contact* contacts, uint32_t* num_contacts);
bool collision_iscolliding_polygonpolygon(Body* a, Body* b, int a_index, int b_index, Contact* contacts, uint32_t* num_contacts);
bool collision_iscolliding_polygoncircle(Body* polygon, Body* circle, int polygon_index, int circle_index, Contact* contacts, uint32_t* num_contacts);

#endif // COLLISION_H
