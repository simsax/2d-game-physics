#include "constraint.h"
#include "body.h"
#include "matMN.h"
#include "vec2.h"
#include "vecN.h"
#include "world.h"
#include "utils.h"
#include <math.h>

JointConstraint constraint_joint_create(
        World* world, int a_index, int b_index, Vec2 anchor_point) {
    Body* a = &world->bodies.items[a_index];
    Body* b = &world->bodies.items[b_index];
    MatMN jacobian = matMN_create(1, 6);
    VecN lambda = vecN_create(1);
    matMN_zero(jacobian);
    vecN_zero(lambda);
    return (JointConstraint) {
        .world = world,
        .a_index = a_index,
        .b_index = b_index,
        .a_point = body_world_to_local_space(a, anchor_point),
        .b_point = body_world_to_local_space(b, anchor_point),
        .jacobian = jacobian,
        .cached_lambda = lambda
    };
}

PenetrationConstraint constraint_penetration_create(
        World* world, int a_index, int b_index, Vec2 a_collision_point, Vec2 b_collision_point, Vec2 normal) {
    MatMN jacobian = matMN_create(2, 6);
    VecN lambda = vecN_create(2);
    matMN_zero(jacobian);
    vecN_zero(lambda);
    return (PenetrationConstraint) {
        .world = world,
        .a_index = a_index,
        .b_index = b_index,
        .a_collision_point = a_collision_point,
        .b_collision_point = b_collision_point,
        .jacobian = jacobian,
        .cached_lambda = lambda,
        .normal = normal
    };
}

void constraint_joint_free(JointConstraint* constraint) {
    matMN_free(constraint->jacobian);
    vecN_free(constraint->cached_lambda);
}

void constraint_penetration_free(PenetrationConstraint* constraint) {
    matMN_free(constraint->jacobian);
    vecN_free(constraint->cached_lambda);
}

// [ 1/ma  0     0     0     0     0 ]
// [ 0     1/ma  0     0     0     0 ]
// [ 0     0     1/Ia  0     0     0 ]
// [ 0     0     0     1/mb  0     0 ]
// [ 0     0     0     0     1/mb  0 ]
// [ 0     0     0     0     0     1/Ib ]
MatMN constraint_joint_get_inv_mass(JointConstraint* constraint) {
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

MatMN constraint_penetration_get_inv_mass(PenetrationConstraint* constraint) {
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
VecN constraint_joint_get_velocities(JointConstraint* constraint) {
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

VecN constraint_penetration_get_velocities(PenetrationConstraint* constraint) {
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

void constraint_joint_pre_solve(JointConstraint* constraint, float dt) {
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
    float beta = 0.2f;
    float C = vec2_dot(pb_pa, pb_pa); // positional error
    C = fmax(C, 0);
    constraint->bias = (beta / dt) * C;
}

void constraint_joint_solve(JointConstraint* constraint) {
    Body* a = &constraint->world->bodies.items[constraint->a_index];
    Body* b = &constraint->world->bodies.items[constraint->b_index];
    MatMN jacobian = constraint->jacobian;
    MatMN inv_mass = constraint_joint_get_inv_mass(constraint);
    MatMN jacobian_t = matMN_transpose(jacobian);
    MatMN j_inv_mass = matMN_mult_mat(jacobian, inv_mass);
    MatMN lhs = matMN_mult_mat(j_inv_mass, jacobian_t); // A
    // TODO: lhs in the presolve (member variable of struct)

    VecN velocities = constraint_joint_get_velocities(constraint);
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


void constraint_joint_post_solve(JointConstraint* constraint) {

}

void constraint_penetration_pre_solve(PenetrationConstraint* constraint, float dt) {
    Body* a = &constraint->world->bodies.items[constraint->a_index];
    Body* b = &constraint->world->bodies.items[constraint->b_index];

    // get collision points
    Vec2 pa = constraint->a_collision_point;
    Vec2 pb = constraint->b_collision_point;

    Vec2 ra = vec2_sub(pa, a->position);
    Vec2 rb = vec2_sub(pb, b->position);
    Vec2 normal = constraint->normal;

    MatMN jacobian = constraint->jacobian;

    // A linear velocity
    Vec2 j1 = vec2_mult(normal, -1.0f);
    MAT_SET(jacobian, 0, 0, j1.x);
    MAT_SET(jacobian, 0, 1, j1.y);

    // A angular velocity
    float j2 = vec2_cross(ra, normal) * -1.0f;
    MAT_SET(jacobian, 0, 2, j2);

    // B linear velocity
    Vec2 j3 = normal;
    MAT_SET(jacobian, 0, 3, j3.x);
    MAT_SET(jacobian, 0, 4, j3.y);

    // B angular velocity
    float j4 = vec2_cross(rb, normal);
    MAT_SET(jacobian, 0, 5, j4);

    // populate second row of the Jacobian (tangent vector - friction)
    constraint->friction = a->friction * b->friction;
    if (constraint->friction > 0.0f) {
        Vec2 t = vec2_normal(normal);
        MAT_SET(jacobian, 1, 0, -t.x);
        MAT_SET(jacobian, 1, 1, -t.y);
        MAT_SET(jacobian, 1, 2, vec2_cross(ra, t) * -1.0f);
        MAT_SET(jacobian, 1, 3, t.x);
        MAT_SET(jacobian, 1, 4, t.y);
        MAT_SET(jacobian, 1, 5, vec2_cross(rb, t)); // TODO: this is the term causing weird bouncy behavior
    }

    // warm starting (apply cached lambda)
    MatMN jacobian_t = matMN_transpose(jacobian);
    VecN impulses = matMN_mult_vec(jacobian_t, constraint->cached_lambda);
    body_apply_impulse_linear(a, VEC2(impulses.data[0], impulses.data[1]));
    body_apply_impulse_angular(a, impulses.data[2]);
    body_apply_impulse_linear(b, VEC2(impulses.data[3], impulses.data[4]));
    body_apply_impulse_angular(b, impulses.data[5]);

    // compute bias term (baumgarte stabilization)
    float beta = 0.2f;
    Vec2 pb_pa = vec2_sub(pb, pa);
    float C = vec2_dot(pb_pa, vec2_mult(normal, -1)); // positional error
    C = fmin(C + 0.01f, 0); // TODO: do we need this?

    Vec2 va = vec2_add(a->velocity, VEC2(-a->angular_velocity * ra.y, a->angular_velocity * ra.x));
    Vec2 vb = vec2_add(b->velocity, VEC2(-b->angular_velocity * rb.y, b->angular_velocity * rb.x));
    float vrel_dot_normal = vec2_dot(vec2_sub(va, vb), normal);

    float e = a->restitution * b->restitution;

    // TODO: penetration and restitution slop
    constraint->bias = (beta / dt) * C + e * vrel_dot_normal;

    MatMN inv_mass = constraint_penetration_get_inv_mass(constraint);
    MatMN j_inv_mass = matMN_mult_mat(jacobian, inv_mass);
    constraint->lhs = matMN_mult_mat(j_inv_mass, jacobian_t);

    matMN_free(jacobian_t);
    matMN_free(inv_mass);
    matMN_free(j_inv_mass);
}

// TODO: num_contacts is just for debugging
void constraint_penetration_solve(PenetrationConstraint* constraint) {
    Body* a = &constraint->world->bodies.items[constraint->a_index];
    Body* b = &constraint->world->bodies.items[constraint->b_index];
    MatMN jacobian = constraint->jacobian;

    VecN velocities = constraint_penetration_get_velocities(constraint);
    VecN j_v = matMN_mult_vec(jacobian, velocities);
    VecN rhs = vecN_mult(j_v, -1.0f); // B (1x2)
    rhs.data[0] -= constraint->bias;

    // Computing lambda by solving 2x2 system directly
    MatMN A = constraint->lhs;
    float det_A = MAT_GET(A, 0, 0) * MAT_GET(A, 1, 1) - MAT_GET(A, 1, 0) * MAT_GET(A, 0, 1);
    /*matMN_print(A);*/

    VecN lambda = vecN_create(2); // TODO: use vec2
    if (constraint->friction) {
        if (det_A == 0.0f) { // TODO: is this comparison legit
            vecN_zero(lambda);
        } else {
            lambda.data[0] = (MAT_GET(A, 1, 1) * rhs.data[0] - MAT_GET(A, 0, 1) * rhs.data[1]) / det_A;
            lambda.data[1] = (MAT_GET(A, 0, 0) * rhs.data[1] - MAT_GET(A, 1, 0) * rhs.data[0]) / det_A;
        }
    } else {
        // if there is no friction, only first term of lhs matrix is meaningful
        float first_term = MAT_GET(A, 0, 0);
        if (first_term != 0.0f) { // TODO: is this comparison legit
            lambda.data[0] = rhs.data[0] / first_term;
        }
    }

    // clamp lambda
    VecN old_cached_lambda = constraint->cached_lambda;
    constraint->cached_lambda = vecN_add(constraint->cached_lambda, lambda);
    constraint->cached_lambda.data[0] = (constraint->cached_lambda.data[0] < 0.0f) ? 0.0f : constraint->cached_lambda.data[0];

    // keep friction values between -λn*μ and λn*μ
    if (constraint->friction) {
        float max_friction = constraint->cached_lambda.data[0] * constraint->friction; // λn*μ
        constraint->cached_lambda.data[1] = clamp(constraint->cached_lambda.data[1], -max_friction, max_friction);
    }

    lambda = vecN_sub(constraint->cached_lambda, old_cached_lambda);

    // compute final impulses with direction and magnitude
    MatMN jacobian_t = matMN_transpose(jacobian);
    VecN impulses = matMN_mult_vec(jacobian_t, lambda);
    /*if (num_contacts == 2)*/
    /*vecN_print(impulses);*/
    body_apply_impulse_linear(a, VEC2(impulses.data[0], impulses.data[1]));
    body_apply_impulse_angular(a, impulses.data[2]);
    body_apply_impulse_linear(b, VEC2(impulses.data[3], impulses.data[4]));
    body_apply_impulse_angular(b, impulses.data[5]);

    // TODO: avoid heap allocations or use a bump allocator
    vecN_free(old_cached_lambda);
    vecN_free(velocities);
    matMN_free(jacobian_t);
    vecN_free(j_v);
    vecN_free(rhs);
    vecN_free(lambda);
    vecN_free(impulses);
}

void constraint_penetration_post_solve(PenetrationConstraint* constraint) {

}

