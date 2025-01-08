#ifndef MANIFOLD_H
#define MANIFOLD_H

#include "constraint.h"
#include "contact.h"

#define MAX_CONTACTS 2

typedef struct {
    int num_contacts;
    PenetrationConstraint constraints[MAX_CONTACTS];
} Manifold;

typedef struct {
    uint32_t capacity;
    uint32_t count;
    Manifold* items;
} ManifoldArray;

Manifold manifold_create(int num_contacts);
bool manifold_contact_almost_equal(Manifold* manifold, Contact* contacts, int num_contacts);
void manifold_free(Manifold* manifold);
void manifold_pre_solve(Manifold* manifold, float dt);
void manifold_solve(Manifold* manifold);
void manifold_post_solve(Manifold* manifold);

#endif // MANIFOLD_H
