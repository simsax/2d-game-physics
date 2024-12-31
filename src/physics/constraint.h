#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "body.h"
#include "matMN.h"

typedef enum {
    JOINT_CONSTRAINT,
    PENETRATION_CONSTRAINT
} ConstraintType;

typedef struct {
    ConstraintType type;
    Body* a;
    Body* b;
    Vec2 a_point; // anchor point in A's local space
    Vec2 b_point; // anchor point in B's local space
    MatMN jacobian;
} Constraint;

Constraint constraint_create(Body* a, Body* b, Vec2 anchor_point);
MatMN constraint_get_inv_mass(Constraint* constraint);
VecN constraint_get_velocities(Constraint* constraint);
void constraint_solve(Constraint* constraint);

#endif // CONSTRAINT_H
