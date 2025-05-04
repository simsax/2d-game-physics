#include "constraint.h"
#include "body.h"
#include "vec2.h"
#include "utils.h"
#include <math.h>

void constraint_joint_init(JointConstraint* constraint, Body* a, Body* b, int a_index, int b_index, Vec2 anchor_point) {
    constraint->a_index = a_index;
    constraint->b_index = b_index;
    constraint->a_point = body_world_to_local_space(a, anchor_point);
    constraint->b_point = body_world_to_local_space(b, anchor_point);
    constraint->lambda = 0;
    constraint->bias = 0;
    constraint->k = 0;
}

void constraint_penetration_init(PenetrationConstraint* constraint, Vec2 a_collision_point, Vec2 b_collision_point, Vec2 normal, bool persistent) {
    constraint->a_collision_point = a_collision_point;
    constraint->b_collision_point = b_collision_point;
    constraint->normal = normal;
    if (!persistent) {
        constraint->lambda_normal = 0;
        constraint->lambda_tangent = 0;
    }
    // otherwise, we re-use the previous impulse
}

void constraint_joint_pre_solve(JointConstraint* constraint, Body* a, Body* b, float dt) {
    // get anchor point position in world space
    Vec2 pa = body_local_to_world_space(a, constraint->a_point);
    Vec2 pb = body_local_to_world_space(b, constraint->b_point);

    Vec2 ra = vec2_sub(pa, a->position);
    Vec2 rb = vec2_sub(pb, b->position);

    Vec2 pa_pb = vec2_sub(pa, pb);
    Vec2 pb_pa = vec2_sub(pb, pa);

    float ra_cross_pab = vec2_cross(ra, pa_pb);
    float rb_cross_pba = vec2_cross(rb, pb_pa);

    float ka = vec2_dot(pa_pb, pa_pb) * a->inv_mass + ra_cross_pab * ra_cross_pab * a->inv_I;
    float kb = vec2_dot(pb_pa, pb_pa) * b->inv_mass + rb_cross_pba * rb_cross_pba * b->inv_I;

    constraint->k = 4 * (ka + kb);

    // warm starting (apply cached lambda)
    float lambda = constraint->lambda;

    Vec2 impulse_linear_a = vec2_mult(pa_pb, 2 * lambda);
    float impulse_angular_a = 2 * ra_cross_pab * lambda;
    Vec2 impulse_linear_b = vec2_mult(pb_pa, 2 * lambda);
    float impulse_angular_b = 2 * rb_cross_pba * lambda;

    body_apply_impulse_linear(a, impulse_linear_a);
    body_apply_impulse_angular(a, impulse_angular_a);
    body_apply_impulse_linear(b, impulse_linear_b);
    body_apply_impulse_angular(b, impulse_angular_b);

    // compute bias term (baumgarte stabilization)
    float beta = 0.2f;
    float C = vec2_dot(pb_pa, pb_pa); // positional error
    C = fmax(C, 0);
    constraint->bias = (beta / dt) * C;
}

void constraint_joint_solve(JointConstraint* constraint, Body* a, Body* b) {
    Vec2 pa = body_local_to_world_space(a, constraint->a_point);
    Vec2 pb = body_local_to_world_space(b, constraint->b_point);

    Vec2 ra = vec2_sub(pa, a->position);
    Vec2 rb = vec2_sub(pb, b->position);

    Vec2 pa_pb = vec2_sub(pa, pb);
    Vec2 pb_pa = vec2_sub(pb, pa);

    float ra_cross_pab = vec2_cross(ra, pa_pb);
    float rb_cross_pba = vec2_cross(rb, pb_pa);

    float j_va = vec2_dot(pa_pb, a->velocity) + ra_cross_pab * a->angular_velocity;
    float j_vb = vec2_dot(pb_pa, b->velocity) + rb_cross_pba * b->angular_velocity;
    float j_v = 2 * (j_va + j_vb);

    float lambda = constraint->k == 0 ? 0 : -(j_v + constraint->bias) / constraint->k;
    constraint->lambda += lambda;

    Vec2 impulse_linear_a = vec2_mult(pa_pb, 2 * lambda);
    float impulse_angular_a = 2 * ra_cross_pab * lambda;
    Vec2 impulse_linear_b = vec2_mult(pb_pa, 2 * lambda);
    float impulse_angular_b = 2 * rb_cross_pba * lambda;

    body_apply_impulse_linear(a, impulse_linear_a);
    body_apply_impulse_angular(a, impulse_angular_a);
    body_apply_impulse_linear(b, impulse_linear_b);
    body_apply_impulse_angular(b, impulse_angular_b);
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

    float lambda_normal = constraint->k_normal == 0 ? 0 : -(vrel_n + constraint->bias) / constraint->k_normal;

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


