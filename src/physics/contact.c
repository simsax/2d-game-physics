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

    Vec2 ra_post = vec_sub(contact->end, a->position);
    Vec2 rb_post = vec_sub(contact->start, b->position);

    // apply velocity correction using impulse method
    float e = fmin(a->restitution, b->restitution);

    // a and b position are changed after the resolve penetration
    // which means this is not that accurate, should probably calculate it before resolve penetration
    // or I update start and end position inside resolve penetration
    Vec2 va = vec_add(a->velocity, VEC2(-a->angular_velocity * ra.y, a->angular_velocity * ra.x));
    Vec2 vb = vec_add(b->velocity, VEC2(-b->angular_velocity * rb.y, b->angular_velocity * rb.x));
    Vec2 vrel = vec_sub(va, vb);

    float vrel_dot_normal = vec_dot(vrel, contact->normal);
    float ra_cross_n = vec_cross(ra, contact->normal);
    float rb_cross_n = vec_cross(rb, contact->normal);
    float denominator_linear = a->inv_mass + b->inv_mass;
    float denominator_angular = ra_cross_n * ra_cross_n * a->inv_I + rb_cross_n * rb_cross_n * b->inv_I;
    float impulse_magnitude = -(1 + e) * vrel_dot_normal / (denominator_linear + denominator_angular);

    Vec2 jn = vec_mult(contact->normal, impulse_magnitude);

    body_apply_impulse(a, jn, ra);
    body_apply_impulse(b, vec_mult(jn, -1), rb);
}
