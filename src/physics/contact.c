#include "contact.h"

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
