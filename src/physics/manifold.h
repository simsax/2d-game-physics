#ifndef MANIFOLD_H
#define MANIFOLD_H

#include "constraint.h"
#include "collision.h"

#define MAX_CONTACTS 2

// TODO: it should be a hastable
// To find if there is already a contact between a and b, should be able to just 
// look for the entry corresponding to a and b, instead of looping every time the entire array

typedef struct {
    int a_index; // index of Body A in world's array (TODO: if we remove a body, this might not be true anymore)
    int b_index; // index of Body B in world's array
    int num_contacts;
    bool expired;
    PenetrationConstraint constraints[MAX_CONTACTS];
} Manifold;

typedef struct {
    uint32_t capacity;
    uint32_t count;
    Manifold* items;
} ManifoldArray;

struct World;

void manifold_init(Manifold* manifold, int num_contacts, int a_index, int b_index);
void manifold_find_existing_contact(Manifold* manifold, bool* persistent, Contact* contact, int* p);
void manifold_pre_solve(Manifold* manifold, BodyArray world_bodies, float dt);
void manifold_solve(Manifold* manifold, BodyArray world_bodies);
void manifold_post_solve(Manifold* manifold);

#endif // MANIFOLD_H
