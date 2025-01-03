#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "body.h"
#include "matMN.h"

typedef enum {
    JOINT_CONSTRAINT,
    PENETRATION_CONSTRAINT
} ConstraintType;

typedef struct World World;

typedef struct {
    ConstraintType type;
    // NOTE: after realloc these body pointers are invalidated since the bodies are allocated directly 
    // in the array (not their pointers). Use indices instead
    /*Body* a;*/
    /*Body* b;*/
    World* world;
    int a_index; // index of body A in the world's bodies array
    int b_index; // index of body B in the world's bodies array
    Vec2 a_point; // anchor point in A's local space
    Vec2 b_point; // anchor point in B's local space
    MatMN jacobian;
    VecN cached_lambda;
    float bias;
} Constraint;

typedef struct {
    uint32_t capacity;
    uint32_t count;
    Constraint* items;
} ConstraintArray;

Constraint constraint_create(ConstraintType type, World* world, int index_a, int index_b, Vec2 anchor_point);
void constraint_free(Constraint* constraint);
MatMN constraint_get_inv_mass(Constraint* constraint);
VecN constraint_get_velocities(Constraint* constraint);
void constraint_solve(Constraint* constraint);
void constraint_pre_solve(Constraint* constraint, float dt);
void constraint_post_solve(Constraint* constraint);

#endif // CONSTRAINT_H
