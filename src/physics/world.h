#ifndef WORLD_H
#define WORLD_H

#include "body.h"
#include "array.h"

typedef struct {
    float gravity;
    BodyArray bodies;
    Vec2Array forces;
    FloatArray torques;
} World;

World world_create(float gravity);
void world_free(World* world);
Body* world_new_body(World* world);
void world_add_force(World* world, Vec2 force);
void world_add_torque(World* world, float torque);
void world_update(World* world, float dt);
void world_check_collisions(World* world);

#endif // WORLD_H
