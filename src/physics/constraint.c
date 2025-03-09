#include "constraint.h"
#include "body.h"
#include "matMN.h"
#include "vec2.h"
#include "vecN.h"
#include "utils.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

JointConstraint constraint_joint_create(
        Body* a, Body* b, int a_index, int b_index, Vec2 anchor_point) {
    MatMN jacobian = matMN_create(1, 6, NULL);
    VecN lambda = vecN_create(1, NULL);
    matMN_zero(jacobian);
    vecN_zero(lambda);
    return (JointConstraint) {
        .a_index = a_index,
        .b_index = b_index,
        .a_point = body_world_to_local_space(a, anchor_point),
        .b_point = body_world_to_local_space(b, anchor_point),
        .jacobian = jacobian,
        .cached_lambda = lambda
    };
}

void constraint_penetration_init(PenetrationConstraint* constraint, int a_index, int b_index, Vec2 a_collision_point, Vec2 b_collision_point, Vec2 normal, bool persistent) {
    constraint->a_collision_point = a_collision_point;
    constraint->b_collision_point = b_collision_point;
    constraint->normal = normal;
    constraint->a_index = a_index;
    constraint->b_index = b_index;
    if (!persistent) {
        constraint->lambda_normal = 0;
        constraint->lambda_tangent = 0;
    }
    // otherwise, we re-use the previous impulse
}

void constraint_joint_free(JointConstraint* constraint) {
    (void) constraint;
}

// [ 1/ma  0     0     0     0     0 ]
// [ 0     1/ma  0     0     0     0 ]
// [ 0     0     1/Ia  0     0     0 ]
// [ 0     0     0     1/mb  0     0 ]
// [ 0     0     0     0     1/mb  0 ]
// [ 0     0     0     0     0     1/Ib ]
static MatMN constraint_joint_get_inv_mass(Body* a, Body* b) {
    MatMN inv_mass = matMN_create(6, 6, NULL);
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
static VecN constraint_joint_get_velocities(Body* a, Body* b) {
    VecN v = vecN_create(6, NULL);
    v.data[0] = a->velocity.x;
    v.data[1] = a->velocity.y;
    v.data[2] = a->angular_velocity;
    v.data[3] = b->velocity.x;
    v.data[4] = b->velocity.y;
    v.data[5] = b->angular_velocity;
    return v;
}

void constraint_joint_pre_solve(JointConstraint* constraint, Body* a, Body* b, float dt) {
    // TODO: temporary hack
    Arena* arena = NULL;

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
    MatMN jacobian_t = matMN_transpose(jacobian, arena);
    VecN impulses = matMN_mult_vec(jacobian_t, constraint->cached_lambda, arena);
    body_apply_impulse_linear(a, VEC2(impulses.data[0], impulses.data[1]));
    body_apply_impulse_angular(a, impulses.data[2]);
    body_apply_impulse_linear(b, VEC2(impulses.data[3], impulses.data[4]));
    body_apply_impulse_angular(b, impulses.data[5]);

    // compute bias term (baumgarte stabilization)
    float beta = 0.2f;
    float C = vec2_dot(pb_pa, pb_pa); // positional error
    C = fmax(C, 0);
    constraint->bias = (beta / dt) * C;
}

void constraint_joint_solve(JointConstraint* constraint, Body* a, Body* b) {
    Arena* arena = NULL;
    MatMN jacobian = constraint->jacobian;
    MatMN inv_mass = constraint_joint_get_inv_mass(a, b);
    MatMN jacobian_t = matMN_transpose(jacobian, arena);
    MatMN j_inv_mass = matMN_mult_mat(jacobian, inv_mass, arena);
    MatMN lhs = matMN_mult_mat(j_inv_mass, jacobian_t, arena); // A
    // TODO: lhs in the presolve (member variable of struct)

    VecN velocities = constraint_joint_get_velocities(a, b);
    VecN j_v = matMN_mult_vec(jacobian, velocities, arena);
    VecN rhs = vecN_mult(j_v, -1.0f, arena); // B
    rhs.data[0] -= constraint->bias;

    // TODO: we end up with 1x1 matrices on numerator, we can solve the system trivially
    /*float denominator = MAT_GET(lhs, 0, 0);*/
    /*VecN lambda = vecN_mult(rhs, denominator == 0.0f ? 1.0f : (1.0f/denominator));*/
    VecN lambda = matMN_solve_gauss_seidel(lhs, rhs, arena);
    VecN old_cached_lambda = constraint->cached_lambda;
    constraint->cached_lambda = vecN_add(constraint->cached_lambda, lambda, arena);

    // compute final impulses with direction and magnitude
    VecN impulses = matMN_mult_vec(jacobian_t, lambda, arena);
    body_apply_impulse_linear(a, VEC2(impulses.data[0], impulses.data[1]));
    body_apply_impulse_angular(a, impulses.data[2]);
    body_apply_impulse_linear(b, VEC2(impulses.data[3], impulses.data[4]));
    body_apply_impulse_angular(b, impulses.data[5]);

    vecN_free(old_cached_lambda);
}


void constraint_penetration_pre_solve(PenetrationConstraint* constraint, Body* a, Body* b, float dt) {
    Vec2 pa = constraint->a_collision_point;
    Vec2 pb = constraint->b_collision_point;

    Vec2 ra = vec2_sub(pa, a->position);
    Vec2 rb = vec2_sub(pb, b->position);
    Vec2 normal = constraint->normal;
    float ra_cross_n = vec2_cross(ra, normal);
    float rb_cross_n = vec2_cross(rb, normal);

    // this formula is obtained by deriving J * M^(-1) * Jt
    float k_a_n = a->inv_mass + a->inv_I * (ra_cross_n * ra_cross_n);
    float k_b_n = b->inv_mass + b->inv_I * (rb_cross_n * rb_cross_n);
    constraint->k_normal = k_a_n + k_b_n;

    constraint->friction = a->friction * b->friction;
    Vec2 tangent = vec2_normal(normal);
    float ra_cross_t = vec2_cross(ra, tangent);
    float rb_cross_t = vec2_cross(rb, tangent);
    float k_a_t = a->inv_mass + a->inv_I * (ra_cross_t * ra_cross_t);
    float k_b_t = b->inv_mass + b->inv_I * (rb_cross_t * rb_cross_t);
    constraint->k_tangent = k_a_t + k_b_t;

    // warm starting
    Vec2 accumulated_impulse = vec2_add(vec2_mult(normal, constraint->lambda_normal), vec2_mult(tangent, constraint->lambda_tangent));
    body_apply_impulse_linear(a, vec2_mult(accumulated_impulse, -1));
    body_apply_impulse_angular(a, -vec2_cross(ra, accumulated_impulse));
    body_apply_impulse_linear(b, accumulated_impulse);
    body_apply_impulse_angular(b, vec2_cross(rb, accumulated_impulse));

    // compute bias term (baumgarte stabilization)
    float beta = 0.1f;
    float penetration_slop = 0.005f;
    float restitution_slop = 0.5f; // 0.5 m/s
    Vec2 pb_pa = vec2_sub(pb, pa);
    float C = vec2_dot(pb_pa, normal); // positional error
    // C is always < 0
    C = fmin(C + penetration_slop, 0);

    Vec2 va = vec2_add(a->velocity, VEC2(-a->angular_velocity * ra.y, a->angular_velocity * ra.x));
    Vec2 vb = vec2_add(b->velocity, VEC2(-b->angular_velocity * rb.y, b->angular_velocity * rb.x));
    float vrel_n = vec2_dot(vec2_sub(vb, va), normal);
    // TODO: check this out
    if (fabsf(vrel_n) <= restitution_slop)
        vrel_n = 0;
    float e = a->restitution * b->restitution;

    constraint->bias = (beta / dt) * C + e * vrel_n;
}

void constraint_penetration_solve(PenetrationConstraint* constraint, Body* a, Body* b) {
    Vec2 pa = constraint->a_collision_point;
    Vec2 pb = constraint->b_collision_point;
    Vec2 ra = vec2_sub(pa, a->position);
    Vec2 rb = vec2_sub(pb, b->position);
    Vec2 normal = constraint->normal;

    float ra_cross_n = vec2_cross(ra, normal);
    float rb_cross_n = vec2_cross(rb, normal);

    float va_n = vec2_dot(a->velocity, normal) + ra_cross_n * a->angular_velocity;
    float vb_n = vec2_dot(b->velocity, normal) + rb_cross_n * b->angular_velocity;
    float vrel_n = vb_n - va_n;

    float lambda_normal = -(vrel_n + constraint->bias) / constraint->k_normal;

    // clamp lambda
    float old_lambda_normal = constraint->lambda_normal;
    constraint->lambda_normal += lambda_normal;
    // clamp to avoid pulling objects together
    if (constraint->lambda_normal < 0.0f)
        constraint->lambda_normal = 0.0f;

    lambda_normal = constraint->lambda_normal - old_lambda_normal;
    body_apply_impulse_linear(a, VEC2(-normal.x * lambda_normal, -normal.y * lambda_normal));
    body_apply_impulse_angular(a, -ra_cross_n * lambda_normal);
    body_apply_impulse_linear(b, VEC2(normal.x * lambda_normal, normal.y * lambda_normal));
    body_apply_impulse_angular(b, rb_cross_n * lambda_normal);

    Vec2 tangent = vec2_normal(normal);
    float ra_cross_t = vec2_cross(ra, tangent);
    float rb_cross_t = vec2_cross(rb, tangent);

    float va_t = vec2_dot(a->velocity, tangent) + ra_cross_t * a->angular_velocity;
    float vb_t = vec2_dot(b->velocity, tangent) + rb_cross_t * b->angular_velocity;
    float vrel_t = vb_t - va_t;
    float lambda_tangent = -vrel_t / constraint->k_tangent;

    // clamp friction between -λn*μ and λn*μ
    float max_friction = constraint->lambda_normal * constraint->friction; // λn*μ
    float old_lambda_tangent = constraint->lambda_tangent;
    constraint->lambda_tangent = clamp(constraint->lambda_tangent + lambda_tangent, -max_friction, max_friction);
    lambda_tangent = constraint->lambda_tangent - old_lambda_tangent;

    body_apply_impulse_linear(a, VEC2(-tangent.x * lambda_tangent, -tangent.y * lambda_tangent));
    body_apply_impulse_angular(a, -ra_cross_t * lambda_tangent);
    body_apply_impulse_linear(b, VEC2(tangent.x * lambda_tangent, tangent.y * lambda_tangent));
    body_apply_impulse_angular(b, rb_cross_t * lambda_tangent);
}


