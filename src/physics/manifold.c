#include "manifold.h"
#include "constraint.h"
#include "vec2.h"

void manifold_init(Manifold* manifold, int num_contacts, int a_index, int b_index) {
    manifold->a_index = a_index;
    manifold->b_index = b_index;
    manifold->num_contacts = num_contacts;
}

bool manifold_find_existing_contact(Manifold* manifold, Contact* contact) {
    float distance_threshold = 0.005f; // 5 mm
    for (int i = 0; i < manifold->num_contacts; i++) {
        PenetrationConstraint* constraint = &manifold->constraints[i];
        Vec2 a_point_m = constraint->a_collision_point;
        Vec2 b_point_m = constraint->b_collision_point;
        Vec2 a_point_c = contact->end;
        Vec2 b_point_c = contact->start;

        if (vec2_magnitude_squared(vec2_sub(a_point_m, a_point_c)) <= distance_threshold * distance_threshold &&
            vec2_magnitude_squared(vec2_sub(b_point_m, b_point_c)) <= distance_threshold * distance_threshold) {
            // found existing contact
            return true;
        }
    }
    return false;
}

void manifold_pre_solve(Manifold* manifold, BodyArray world_bodies, float dt) {
    for (int i = 0; i < manifold->num_contacts; i++) {
        PenetrationConstraint* constraint = &manifold->constraints[i];
        Body* a = &world_bodies.items[constraint->a_index];
        Body* b = &world_bodies.items[constraint->b_index];
        constraint_penetration_pre_solve(constraint, a, b, dt);
    }
}

void manifold_solve(Manifold* manifold, BodyArray world_bodies) {
    for (int i = 0; i < manifold->num_contacts; i++) {
        PenetrationConstraint* constraint = &manifold->constraints[i];
        Body* a = &world_bodies.items[constraint->a_index];
        Body* b = &world_bodies.items[constraint->b_index];
        constraint_penetration_solve(constraint, a, b);
    }
}

