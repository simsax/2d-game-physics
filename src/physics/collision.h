#ifndef COLLISION_H
#define COLLISION_H

#include <stdbool.h>
#include "body.h"

bool collision_iscolliding(Body* a, Body* b);
bool collision_iscolliding_circlecircle(Body* a, Body* b);
/*bool collision_iscolliding_polygonpolygon(Body* a, Body* b);*/
/*bool collision_iscolliding_polygoncircle(Body* a, Body* b);*/

#endif // COLLISION_H
