#include "constraint.h"
#include "body.h"
#include "matMN.h"
#include "vec2.h"
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

void constraint_free(Constraint* constraint) {
    matMN_free(constraint->jacobian);
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
    MAT_SET(inv_mass, 3, 3, constraint->b->inv_mass);
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
    // get anchor point position in world space
    Vec2 pa = body_local_to_world_space(constraint->a, constraint->a_point);
    Vec2 pb = body_local_to_world_space(constraint->b, constraint->b_point);

    Vec2 ra = vec2_sub(pa, constraint->a->position);
    Vec2 rb = vec2_sub(pb, constraint->b->position);

    MatMN jacobian = constraint->jacobian;

    // A linear velocity
    Vec2 pa_pb = vec2_sub(pa, pb);
    Vec2 j1 = vec2_mult(pa_pb, 2.0f);
    MAT_SET(jacobian, 0, 0, j1.x);
    MAT_SET(jacobian, 0, 1, j1.y);

    // A angular velocity
    float j2 = vec2_cross(ra, pa_pb) * 2.0f;
    MAT_SET(jacobian, 0, 2, j2);

    // B linear velocity
    Vec2 pb_pa = vec2_sub(pb, pa);
    Vec2 j3 = vec2_mult(pb_pa, 2.0f);
    MAT_SET(jacobian, 0, 3, j3.x);
    MAT_SET(jacobian, 0, 4, j3.y);

    // B angular velocity
    float j4 = vec2_cross(rb, pb_pa) * 2.0f;
    MAT_SET(jacobian, 0, 5, j4);

    VecN velocities = constraint_get_velocities(constraint);
    MatMN inv_mass = constraint_get_inv_mass(constraint);
}

