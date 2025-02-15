#include "manifold.h"
#include "constraint.h"
#include "vec2.h"
#include <stdio.h>
#include "../graphics.h"

void manifold_init(Manifold* manifold, int num_contacts, int a_index, int b_index) {
    manifold->a_index = a_index;
    manifold->b_index = b_index;
    manifold->num_contacts = num_contacts;
    manifold->expired = false;
}

void manifold_find_existing_contact(Manifold* manifold, bool* persistent, Contact* contact, int* p) {
    float distance_threshold = 0.01f;
    for (int i = 0; i < manifold->num_contacts; i++) {
        PenetrationConstraint* constraint = &manifold->constraints[i];
        Vec2 a_point_m = constraint->a_collision_point;
        Vec2 b_point_m = constraint->b_collision_point;
        Vec2 a_point_c = contact->start;
        Vec2 b_point_c = contact->end;

        if (vec2_magnitude_squared(vec2_sub(a_point_m, a_point_c)) <= distance_threshold * distance_threshold &&
            vec2_magnitude_squared(vec2_sub(b_point_m, b_point_c)) <= distance_threshold * distance_threshold) {
            // found existing contact
            *persistent = true;
            (*p)++;
            /*printf("Found contact! Lambda_zero is (%f, %f)\n", (double)lambda_zero[0], (double)lambda_zero[1]);*/
            /*draw_fill_circle_meters(a_point_m.x, a_point_m.y, 2, 0xFF0000FF);*/
            /*draw_fill_circle_meters(b_point_m.x, b_point_m.y, 2, 0xFF0000FF);*/
            /*draw_fill_circle_meters(a_point_c.x, a_point_c.y, 2, 0x00FF00FF);*/
            /*draw_fill_circle_meters(b_point_c.x, b_point_c.y, 2, 0x00FF00FF);*/
            return;
        }
    }
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

void manifold_post_solve(Manifold* manifold) {
    for (int i = 0; i < manifold->num_contacts; i++) {
        constraint_penetration_post_solve(&manifold->constraints[i]);
    }
}

