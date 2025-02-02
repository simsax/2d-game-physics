#include "manifold.h"
#include "vec2.h"

Manifold manifold_create(int num_contacts, int a_index, int b_index) {
    return (Manifold) {
        .a_index = a_index,
        .b_index = b_index,
        .num_contacts = num_contacts,
        .expired = false
    };
}

bool manifold_contact_almost_equal(Manifold* manifold, Contact* contacts, int num_contacts) {
    if (manifold->num_contacts != num_contacts)
        return false;

    float distance_threshold = 0.01f;
    for (int i = 0; i < num_contacts; i++) {
        Vec2 a_point_m = manifold->constraints[i].a_collision_point;
        Vec2 b_point_m = manifold->constraints[i].b_collision_point;
        Vec2 a_point_c = contacts[i].start;
        Vec2 b_point_c = contacts[i].end;

        if (vec2_magnitude(vec2_sub(a_point_m, a_point_c)) > distance_threshold ||
            vec2_magnitude(vec2_sub(b_point_m, b_point_c)) > distance_threshold) {
            return false;
        }
    }
    return true;
}

void manifold_pre_solve(Manifold* manifold, Body* a, Body* b, float dt) {
    for (int i = 0; i < manifold->num_contacts; i++) {
        constraint_penetration_pre_solve(&manifold->constraints[i], a, b, dt);
    }
}

void manifold_solve(Manifold* manifold, Body* a, Body* b) {
    /*if (manifold->num_contacts == 2)*/
        /*printf("Manifold %d %d\n", manifold->a_index, manifold->b_index);*/
    for (int i = 0; i < manifold->num_contacts; i++) {
        constraint_penetration_solve(&manifold->constraints[i], a, b);
    }
    /*if (manifold->num_contacts == 2)*/
    /*    printf("----------\n");*/
}

void manifold_post_solve(Manifold* manifold) {
    for (int i = 0; i < manifold->num_contacts; i++) {
        constraint_penetration_post_solve(&manifold->constraints[i]);
    }
}

