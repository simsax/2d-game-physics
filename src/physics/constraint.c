#include "constraint.h"
#include "body.h"
#include "matMN.h"
#include "vec2.h"
#include "vecN.h"
#include "world.h"
#include <math.h>

Constraint constraint_create(
        ConstraintType type, World* world, int a_index, int b_index, Vec2 anchor_point) {
    Body* a = &world->bodies.items[a_index];
    Body* b = &world->bodies.items[b_index];
    MatMN jacobian = matMN_create(1, 6);
    VecN lambda = vecN_create(1);
    matMN_zero(jacobian);
    vecN_zero(lambda);
    return (Constraint) {
        .type = type,
        .world = world,
        .a_index = a_index,
        .b_index = b_index,
        .a_point = body_world_to_local_space(a, anchor_point),
        .b_point = body_world_to_local_space(b, anchor_point),
        .jacobian = jacobian,
        .cached_lambda = lambda
    };
}

void constraint_free(Constraint* constraint) {
    matMN_free(constraint->jacobian);
    vecN_free(constraint->cached_lambda);
}

// [ 1/ma  0     0     0     0     0 ]
// [ 0     1/ma  0     0     0     0 ]
// [ 0     0     1/Ia  0     0     0 ]
// [ 0     0     0     1/mb  0     0 ]
// [ 0     0     0     0     1/mb  0 ]
// [ 0     0     0     0     0     1/Ib ]
MatMN constraint_get_inv_mass(Constraint* constraint) {
    Body* a = &constraint->world->bodies.items[constraint->a_index];
    Body* b = &constraint->world->bodies.items[constraint->b_index];
    MatMN inv_mass = matMN_create(6, 6);
    matMN_zero(inv_mass);
    MAT_SET(inv_mass, 0, 0, a->inv_mass);
    MAT_SET(inv_mass, 1, 1, a->inv_mass);
    MAT_SET(inv_mass, 2, 2, a->inv_I);
    MAT_SET(inv_mass, 3, 3, b->inv_mass);
    MAT_SET(inv_mass, 4, 4, b->inv_mass);
    MAT_SET(inv_mass, 5, 5, b->inv_I);
    return inv_mass;
}

// [va.x, va.y, ωa, vb.x, vb.y, ωb]
VecN constraint_get_velocities(Constraint* constraint) {
    Body* a = &constraint->world->bodies.items[constraint->a_index];
    Body* b = &constraint->world->bodies.items[constraint->b_index];
    VecN v = vecN_create(6);
    v.data[0] = a->velocity.x;
    v.data[1] = a->velocity.y;
    v.data[2] = a->angular_velocity;
    v.data[3] = b->velocity.x;
    v.data[4] = b->velocity.y;
    v.data[5] = b->angular_velocity;
    return v;
}

void constraint_pre_solve(Constraint* constraint, float dt) {
    Body* a = &constraint->world->bodies.items[constraint->a_index];
    Body* b = &constraint->world->bodies.items[constraint->b_index];

    // get anchor point position in world space
    Vec2 pa = body_local_to_world_space(a, constraint->a_point);
    Vec2 pb = body_local_to_world_space(b, constraint->b_point);

    Vec2 ra = vec2_sub(pa, a->position);
    Vec2 rb = vec2_sub(pb, b->position);

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

    // warm starting (apply cached lambda)
    MatMN jacobian_t = matMN_transpose(jacobian);
    VecN impulses = matMN_mult_vec(jacobian_t, constraint->cached_lambda);
    body_apply_impulse_linear(a, VEC2(impulses.data[0], impulses.data[1]));
    body_apply_impulse_angular(a, impulses.data[2]);
    body_apply_impulse_linear(b, VEC2(impulses.data[3], impulses.data[4]));
    body_apply_impulse_angular(b, impulses.data[5]);

    matMN_free(jacobian_t);

    // compute bias term (baumgarte stabilization)
    float beta = 0.1f;
    float C = vec2_dot(pb_pa, pb_pa); // positional error
    C = fmax(C, 0);
    constraint->bias = (beta / dt) * C;
}

void constraint_solve(Constraint* constraint) {
    // TODO: IF JOINT...
    Body* a = &constraint->world->bodies.items[constraint->a_index];
    Body* b = &constraint->world->bodies.items[constraint->b_index];
    MatMN jacobian = constraint->jacobian;
    MatMN inv_mass = constraint_get_inv_mass(constraint);
    MatMN jacobian_t = matMN_transpose(jacobian);
    MatMN j_inv_mass = matMN_mult_mat(jacobian, inv_mass);
    MatMN lhs = matMN_mult_mat(j_inv_mass, jacobian_t); // A
    // TODO: lhs in the presolve (member variable of struct)

    VecN velocities = constraint_get_velocities(constraint);
    VecN j_v = matMN_mult_vec(jacobian, velocities);
    VecN rhs = vecN_mult(j_v, -1.0f); // B
    rhs.data[0] -= constraint->bias;

    // TODO: we end up with 1x1 matrices on numerator, we can solve the system trivially
    /*float denominator = MAT_GET(lhs, 0, 0);*/
    /*VecN lambda = vecN_mult(rhs, denominator == 0.0f ? 1.0f : (1.0f/denominator));*/
    VecN lambda = matMN_solve_gauss_seidel(lhs, rhs);
    VecN old_cached_lambda = constraint->cached_lambda;
    constraint->cached_lambda = vecN_add(constraint->cached_lambda, lambda);

    // compute final impulses with direction and magnitude
    VecN impulses = matMN_mult_vec(jacobian_t, lambda);
    body_apply_impulse_linear(a, VEC2(impulses.data[0], impulses.data[1]));
    body_apply_impulse_angular(a, impulses.data[2]);
    body_apply_impulse_linear(b, VEC2(impulses.data[3], impulses.data[4]));
    body_apply_impulse_angular(b, impulses.data[5]);

    // TODO: avoid heap allocations or use a bump allocator
    vecN_free(old_cached_lambda);
    vecN_free(velocities);
    matMN_free(inv_mass);
    matMN_free(jacobian_t);
    matMN_free(j_inv_mass);
    matMN_free(lhs);
    vecN_free(j_v);
    vecN_free(rhs);
    vecN_free(lambda);
    vecN_free(impulses);
}


void constraint_post_solve(Constraint* constraint) {

}

