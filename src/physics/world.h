#ifndef WORLD_H
#define WORLD_H

#include "body.h"
#include "array.h"
#include "constraint.h"
#include "manifold.h"
#include "memory.h"
#include "table.h"

typedef struct World {
    BodyArray bodies;
    /*JointConstraintArray joint_constraints;*/
    Table manifold_map;
    Vec2Array forces;
    FloatArray torques;
    float gravity;
    bool warm_start;
} World;

void world_init(World* world, float gravity);
void world_free(World* world);
Body* world_new_body(World* world);
/*JointConstraint* world_new_joint_constraint(World* world);*/
void world_add_force(World* world, Vec2 force);
void world_add_torque(World* world, float torque);
void world_update(World* world, float dt);
void world_check_collisions(World* world);

#endif // WORLD_H

