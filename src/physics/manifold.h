#ifndef MANIFOLD_H
#define MANIFOLD_H

#include "constraint.h"
#include "collision.h"

#define MAX_CONTACTS 2

typedef struct {
    PenetrationConstraint constraints[MAX_CONTACTS];
    int a_index; // index of Body A in world's array
    int b_index; // index of Body B in world's array
    uint8_t num_contacts; // 0, 1, 2
    bool expired;
} Manifold;

typedef struct {
    uint32_t capacity;
    uint32_t count;
    Manifold* items;
} ManifoldArray;

struct World;

void manifold_init(Manifold* manifold, int num_contacts, int a_index, int b_index);
bool manifold_find_existing_contact(Manifold* manifold, Contact* contact);
void manifold_pre_solve(Manifold* manifold, BodyArray world_bodies, float dt);
void manifold_solve(Manifold* manifold, BodyArray world_bodies);

#endif // MANIFOLD_H
