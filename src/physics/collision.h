#ifndef COLLISION_H
#define COLLISION_H

#include <stdbool.h>
#include "body.h"
#include "contact.h"

bool collision_iscolliding(Body* a, Body* b, int a_index, int b_index, Contact* contacts, int* num_contacts);
bool collision_iscolliding_circlecircle(Body* a, Body* b, int a_index, int b_index, Contact* contacts, int* num_contacts);
bool collision_iscolliding_polygonpolygon(Body* a, Body* b, int a_index, int b_index, Contact* contacts, int* num_contacts);
bool collision_iscolliding_polygoncircle(Body* polygon, Body* circle, int polygon_index, int circle_index, Contact* contacts, int* num_contacts);

#endif // COLLISION_H
