#ifndef FORCE_H
#define FORCE_H

#include "body.h"

Vec2 force_generate_drag(Body* body, float k);
Vec2 force_generate_friction(Body* body, float k);
Vec2 force_generate_gravitational(Body* a, Body* b, float G, float min_dist, float max_dist);
Vec2 force_generate_spring_anchor(Body* body, Vec2 anchor, float rest_length, float k);
Vec2 force_generate_spring_body(Body* a, Body* b, float rest_length, float k);

#endif // FORCE_H
