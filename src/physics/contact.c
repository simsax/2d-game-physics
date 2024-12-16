#include "contact.h"
#include "body.h"
#include "vec2.h"
#include <math.h>

void contact_resolve_penetration(Contact* contact) {
    Body* a = contact->a;
    Body* b = contact->b;

    if (body_is_static(a) && body_is_static(b))
        return;

    float mass_sum = a->inv_mass + b->inv_mass;
    float da = contact->depth * a->inv_mass / mass_sum;
    float db = contact->depth * b->inv_mass / mass_sum;

    a->position = vec_add(a->position, vec_mult(contact->normal, -da));
    b->position = vec_add(b->position, vec_mult(contact->normal, db));
}

void contact_resolve_collision(Contact *contact) {
    Body* a = contact->a;
    Body* b = contact->b;
    Vec2 ra = vec_sub(contact->end, a->position);
    Vec2 rb = vec_sub(contact->start, b->position);

    // apply position correction using projection method
    contact_resolve_penetration(contact);   

    // apply velocity correction using impulse method
    float e = a->restitution * b->restitution;
    float f = a->friction * b->friction;

    Vec2 va = vec_add(a->velocity, VEC2(-a->angular_velocity * ra.y, a->angular_velocity * ra.x));
    Vec2 vb = vec_add(b->velocity, VEC2(-b->angular_velocity * rb.y, b->angular_velocity * rb.x));
    Vec2 vrel = vec_sub(va, vb);

    // calculate collision impulse along the normal
    float vrel_dot_normal = vec_dot(vrel, contact->normal);
    float ra_cross_n = vec_cross(ra, contact->normal);
    float rb_cross_n = vec_cross(rb, contact->normal);
    float denominator_linear = a->inv_mass + b->inv_mass;
    float denominator_angular = ra_cross_n * ra_cross_n * a->inv_I + rb_cross_n * rb_cross_n * b->inv_I;
    float impulse_magnitude = -(1 + e) * vrel_dot_normal / (denominator_linear + denominator_angular);
    Vec2 jn = vec_mult(contact->normal, impulse_magnitude);

    // calculate collision impulse along the tangent
    Vec2 tangent = vec_normal(contact->normal);
    float vrel_dot_tangent = vec_dot(vrel, tangent);
    float ra_cross_t = vec_cross(ra, tangent);
    float rb_cross_t = vec_cross(rb, tangent);
    denominator_angular = ra_cross_t * ra_cross_t * a->inv_I + rb_cross_t * rb_cross_t * b->inv_I;
    float impulse_magnitude_tangent = f * -(1 + e) * vrel_dot_tangent / (denominator_linear + denominator_angular);
    Vec2 jt = vec_mult(tangent, impulse_magnitude_tangent);

    Vec2 j = vec_add(jn, jt);

    body_apply_impulse(a, j, ra);
    body_apply_impulse(b, vec_mult(j, -1), rb);
}
