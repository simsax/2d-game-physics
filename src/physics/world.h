#ifndef WORLD_H
#define WORLD_H

#include "body.h"
#include "array.h"
#include "constraint.h"
#include "manifold.h"

typedef struct World {
    float gravity;
    BodyArray bodies;
    JointConstraintArray joint_constraints;
    ManifoldArray manifolds;
    Vec2Array forces;
    FloatArray torques;
} World;

World world_create(float gravity);
void world_free(World* world);
Body* world_new_body(World* world);
JointConstraint* world_new_joint_constraint(World* world);
void world_add_force(World* world, Vec2 force);
void world_add_torque(World* world, float torque);
void world_update(World* world, float dt);
void world_check_collisions(World* world);
Manifold* world_manifold_next(World* world);
Manifold* world_manifold_find(World* world, int a_index, int b_index);

#endif // WORLD_H

