#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "body.h"
#include "matMN.h"

typedef struct {
    int a_index; // index of body A in the world's bodies array
    int b_index; // index of body B in the world's bodies array
    Vec2 a_point; // anchor point in A's local space
    Vec2 b_point; // anchor point in B's local space
    MatMN jacobian;
    VecN cached_lambda;
    float bias;
} JointConstraint;

typedef struct {
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

JointConstraint constraint_joint_create(Body* a, Body* b, int a_index, int b_index, Vec2 anchor_point);
void constraint_joint_free(JointConstraint* constraint);
void constraint_joint_solve(JointConstraint* constraint, Body* a, Body* b);
void constraint_joint_pre_solve(JointConstraint* constraint, Body* a, Body* b, float dt);
void constraint_joint_post_solve(JointConstraint* constraint);

void constraint_penetration_init(PenetrationConstraint* constraint, int a_index, int b_index, Vec2 a_collision_point, Vec2 b_collision_point, Vec2 normal, float* lambda_zero);
void constraint_penetration_solve(PenetrationConstraint* constraint, Body* a, Body* b);
void constraint_penetration_pre_solve(PenetrationConstraint* constraint, Body* a, Body* b, float dt);
void constraint_penetration_post_solve(PenetrationConstraint* constraint);

#endif // CONSTRAINT_H
