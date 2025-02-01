#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "body.h"
#include "matMN.h"

/*typedef struct World World;*/

typedef struct {
    // NOTE: after realloc these body pointers are invalidated since the bodies are allocated directly 
    // in the array (not their pointers). Use indices instead
    /*Body* a;*/
    /*Body* b;*/
    struct World* world; // TODO: remove world from here?
    int a_index; // index of body A in the world's bodies array
    int b_index; // index of body B in the world's bodies array
    Vec2 a_point; // anchor point in A's local space
    Vec2 b_point; // anchor point in B's local space
    MatMN jacobian;
    VecN cached_lambda;
    float bias;
} JointConstraint;

typedef struct {
    struct World* world; // TODO: remove world from here?
    int a_index; // index of body A in the world's bodies array
    int b_index; // index of body B in the world's bodies array
    Vec2 a_collision_point; // collision point of A in world's space
    Vec2 b_collision_point; // collision point of B in world's space
    float jacobian[2][6];
    float lhs[2][2]; // J*M_inv*Jt
    float cached_lambda[2]; // TODO: use Vec2
    float bias;
    Vec2 normal;
    float friction; // friction coefficient between the two penetrating bodies
} PenetrationConstraint;

typedef struct {
    uint32_t capacity;
    uint32_t count;
    JointConstraint* items;
} JointConstraintArray;

typedef struct {
    uint32_t capacity;
    uint32_t count;
    PenetrationConstraint* items;
} PenetrationConstraintArray;

JointConstraint constraint_joint_create(struct World* world, int a_index, int b_index, Vec2 anchor_point);
void constraint_joint_free(JointConstraint* constraint);
MatMN constraint_joint_get_inv_mass(JointConstraint* constraint);
VecN constraint_joint_get_velocities(JointConstraint* constraint);
void constraint_joint_solve(JointConstraint* constraint);
void constraint_joint_pre_solve(JointConstraint* constraint, float dt);
void constraint_joint_post_solve(JointConstraint* constraint);

PenetrationConstraint constraint_penetration_create(struct World* world, int a_index, int b_index, Vec2 a_collision_point, Vec2 b_collision_point, Vec2 normal);
void constraint_penetration_free(PenetrationConstraint* constraint);
MatMN constraint_penetration_get_inv_mass(PenetrationConstraint* constraint);
VecN constraint_penetration_get_velocities(PenetrationConstraint* constraint);
void constraint_penetration_solve(PenetrationConstraint* constraint);
void constraint_penetration_pre_solve(PenetrationConstraint* constraint, float dt);
void constraint_penetration_post_solve(PenetrationConstraint* constraint);

#endif // CONSTRAINT_H
