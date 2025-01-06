#ifndef COLLISION_H
#define COLLISION_H

#include <stdbool.h>
#include "body.h"
#include "contact.h"

bool collision_iscolliding(Body* a, Body* b, Contact* contacts, int length);
bool collision_iscolliding_circlecircle(Body* a, Body* b, Contact* contacts, int length);
bool collision_iscolliding_polygonpolygon(Body* a, Body* b, Contact* contacts, int length);
bool collision_iscolliding_polygoncircle(Body* polygon, Body* circle, Contact* contacts, int length);

#endif // COLLISION_H
