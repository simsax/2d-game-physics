#include "constraint.h"
#include "matMN.h"
#include "vecN.h"

Constraint constraint_create(Body* a, Body* b, Vec2 anchor_point) {
    MatMN jacobian = matMN_create(1, 6);
    return (Constraint) {
        .a = a,
        .b = b,
        .a_point = body_world_to_local_space(a, anchor_point),
        .b_point = body_world_to_local_space(b, anchor_point),
        .jacobian = jacobian
    };
}

// [ 1/ma  0     0     0     0     0 ]
// [ 0     1/ma  0     0     0     0 ]
// [ 0     0     1/Ia  0     0     0 ]
// [ 0     0     0     1/mb  0     0 ]
// [ 0     0     0     0     1/mb  0 ]
// [ 0     0     0     0     0     1/Ib ]
MatMN constraint_get_inv_mass(Constraint* constraint) {
    MatMN inv_mass = matMN_create(6, 6);
    matMN_zero(inv_mass);
    MAT_SET(inv_mass, 0, 0, constraint->a->inv_mass);
    MAT_SET(inv_mass, 1, 1, constraint->a->inv_mass);
    MAT_SET(inv_mass, 2, 2, constraint->a->inv_I);
    MAT_SET(inv_mass, 3, 4, constraint->b->inv_mass);
    MAT_SET(inv_mass, 4, 4, constraint->b->inv_mass);
    MAT_SET(inv_mass, 5, 5, constraint->b->inv_I);
    return inv_mass;
}

// [va.x, va.y, ωa, vb.x, vb.y, ωb]
VecN constraint_get_velocities(Constraint* constraint) {
    VecN v = vecN_create(6);
    v.data[0] = constraint->a->velocity.x;
    v.data[1] = constraint->a->velocity.y;
    v.data[2] = constraint->a->angular_velocity;
    v.data[3] = constraint->b->velocity.x;
    v.data[4] = constraint->b->velocity.y;
    v.data[5] = constraint->b->angular_velocity;
    return v;
}

void constraint_solve(Constraint* constraint) {

}
