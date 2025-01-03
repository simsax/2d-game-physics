#include "contact.h"
#include "body.h"
#include "shape.h"
#include "vec2.h"

void contact_resolve_penetration(Contact* contact) {
    Body* a = contact->a;
    Body* b = contact->b;

    if (body_is_static(a) && body_is_static(b))
        return;

    float mass_sum = a->inv_mass + b->inv_mass;
    float da = contact->depth * a->inv_mass / mass_sum;
    float db = contact->depth * b->inv_mass / mass_sum;

    a->position = vec2_add(a->position, vec2_mult(contact->normal, -da));
    b->position = vec2_add(b->position, vec2_mult(contact->normal, db));

    shape_update_vertices(&a->shape, a->rotation, a->position);
    shape_update_vertices(&b->shape, b->rotation, b->position);
}

void contact_resolve_collision(Contact *contact) {
    Body* a = contact->a;
    Body* b = contact->b;
    Vec2 ra = vec2_sub(contact->end, a->position);
    Vec2 rb = vec2_sub(contact->start, b->position);

    // apply position correction using projection method
    contact_resolve_penetration(contact);   

    // apply velocity correction using impulse method
    float e = a->restitution * b->restitution;
    float f = a->friction * b->friction;

    Vec2 va = vec2_add(a->velocity, VEC2(-a->angular_velocity * ra.y, a->angular_velocity * ra.x));
    Vec2 vb = vec2_add(b->velocity, VEC2(-b->angular_velocity * rb.y, b->angular_velocity * rb.x));
    Vec2 vrel = vec2_sub(va, vb);

    // calculate collision impulse along the normal
    float vrel_dot_normal = vec2_dot(vrel, contact->normal);
    float ra_cross_n = vec2_cross(ra, contact->normal);
    float rb_cross_n = vec2_cross(rb, contact->normal);
    float denominator_linear = a->inv_mass + b->inv_mass;
    float denominator_angular = ra_cross_n * ra_cross_n * a->inv_I + rb_cross_n * rb_cross_n * b->inv_I;
    float impulse_magnitude = -(1 + e) * vrel_dot_normal / (denominator_linear + denominator_angular);
    Vec2 jn = vec2_mult(contact->normal, impulse_magnitude);

    // calculate collision impulse along the tangent
    Vec2 tangent = vec2_normal(contact->normal);
    float vrel_dot_tangent = vec2_dot(vrel, tangent);
    float ra_cross_t = vec2_cross(ra, tangent);
    float rb_cross_t = vec2_cross(rb, tangent);
    denominator_angular = ra_cross_t * ra_cross_t * a->inv_I + rb_cross_t * rb_cross_t * b->inv_I;
    float impulse_magnitude_tangent = f * -(1 + e) * vrel_dot_tangent / (denominator_linear + denominator_angular);
    Vec2 jt = vec2_mult(tangent, impulse_magnitude_tangent);

    Vec2 j = vec2_add(jn, jt);

    body_apply_impulse_at_point(a, j, ra);
    body_apply_impulse_at_point(b, vec2_mult(j, -1), rb);
}
