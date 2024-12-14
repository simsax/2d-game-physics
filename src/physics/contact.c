#include "contact.h"
#include "body.h"
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
    // apply position correction using projection method
    contact_resolve_penetration(contact);   

    // apply velocity correction using impulse method
    Body* a = contact->a;
    Body* b = contact->b;

    float e = fmin(a->restitution, b->restitution);
    Vec2 vrel = vec_sub(a->velocity, b->velocity);

    float vrel_dot_normal = vec_dot(vrel, contact->normal);
    float impulse_magnitude = -(1 + e) * vrel_dot_normal / (a->inv_mass + b->inv_mass);

    Vec2 j = vec_mult(contact->normal, impulse_magnitude);

    body_apply_impulse(a, j);
    body_apply_impulse(b, vec_mult(j, -1));
}
