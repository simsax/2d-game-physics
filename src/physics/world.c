#include "world.h"
#include "array.h"
#include "constraint.h"
#include "collision.h"
#include "manifold.h"
#include <raylib.h>

#define SOLVE_ITERATIONS 8

void world_init(World* world, float gravity) {
    world->gravity = gravity; // y points down in screen space
    ht_init(&world->manifold_map, 16, 70);
}

void world_free(World* world) {
    for (uint32_t i = 0; i < world->bodies.count; i++) {
        Body* body = &world->bodies.items[i];
        bool is_polygon = body->shape.type == SHAPE_POLYGON || body->shape.type == SHAPE_BOX;
        if (is_polygon) {
            DA_FREE(&body->shape.as.polygon.local_vertices);
            DA_FREE(&body->shape.as.polygon.world_vertices);
            DA_FREE(&body->shape.as.polygon.prev_world_vertices);
        }
    }

    ht_free(&world->manifold_map);
    DA_FREE(&world->joint_constraints);
    DA_FREE(&world->bodies);
    DA_FREE(&world->forces);
    DA_FREE(&world->torques);
}

Body* world_new_body(World* world) {
    return DA_NEXT_PTR(&world->bodies);
}

JointConstraint* world_new_joint(World* world) {
    return DA_NEXT_PTR(&world->joint_constraints);
}

void world_add_force(World* world, Vec2 force) {
    DA_APPEND(&world->forces, force);
}

void world_add_torque(World* world, float torque) {
    DA_APPEND(&world->torques, torque);
}

void world_update(World* world, float dt) {
    // apply all the forces
    for (uint32_t i = 0; i < world->bodies.count; i++) {
        Body* body = &world->bodies.items[i];

        // add weight force
        Vec2 weight = VEC2(0.0,  world->gravity / body->inv_mass);
        body_add_force(body, weight);

        // add forces
        for (uint32_t f = 0;  f < world->forces.count; f++) {
            body_add_force(body, world->forces.items[f]);
        }

        // add torques
        for (uint32_t t = 0;  t < world->torques.count; t++) {
            body_add_torque(body, world->torques.items[t]);
        }
    }

    // integrate all the forces
    for (uint32_t i = 0; i < world->bodies.count; i++) {
        Body* body = &world->bodies.items[i];
        body_integrate_forces(body, dt);
    }

    // check collisions
    for (uint32_t i = 0; i < world->bodies.count - 1; i++) {
        for (uint32_t j = i + 1; j < world->bodies.count; j++) {
            Body* a = &world->bodies.items[i];
            Body* b = &world->bodies.items[j];
            Contact contacts[2];
            uint32_t num_contacts = 0;
            if (collision_iscolliding(a, b, contacts, &num_contacts)) {
                // find if there is already an existing manifold between A and B
                bool persistent[2] = { false };
                bool found = false;
                Manifold* manifold = ht_get_or_new(&world->manifold_map, (Pair){i, j}, num_contacts, &found);
                manifold->expired = false;
                if (found) {
                    // manifold exists, check persistent contacts
                    if (world->warm_start) {
                        for (uint32_t c = 0; c < num_contacts; c++) {
                            persistent[c] = manifold_find_existing_contact(manifold, &contacts[c]);
                        }
                    }
                } 
                for (uint32_t c = 0; c < num_contacts; c++) {
                    // contact->end is pa, contact->start is pb, normal is from A to B
                    constraint_penetration_init(
                        &manifold->constraints[c], contacts[c].end, contacts[c].start, contacts[c].normal, persistent[c]);
                }
                manifold->num_contacts = num_contacts;
            } 
        }
    }

    for (uint32_t c = 0; c < world->joint_constraints.count; c++) {
        JointConstraint* constraint = &world->joint_constraints.items[c];
        Body* a = &world->bodies.items[constraint->a_index];
        Body* b = &world->bodies.items[constraint->b_index];
        constraint_joint_pre_solve(constraint, a, b, dt);
    }

    for (uint32_t c = 0; c < world->manifold_map.capacity; c++) {
        Bucket* bucket = &world->manifold_map.buckets[c];
        if (bucket->occupied) {
            if (!bucket->value.expired) {
                manifold_pre_solve(&world->manifold_map.buckets[c].value, world->bodies, dt);
                bucket->value.expired = true;
            } else {
                ht_remove_bucket(bucket);
            }
        } 
    }
    for (uint32_t i = 0; i < SOLVE_ITERATIONS; i++) {
        // joints
        for (uint32_t c = 0; c < world->joint_constraints.count; c++) {
            JointConstraint* constraint = &world->joint_constraints.items[c];
            Body* a = &world->bodies.items[constraint->a_index];
            Body* b = &world->bodies.items[constraint->b_index];
            constraint_joint_solve(constraint, a, b);
        }
        // penetrations
        for (uint32_t c = 0; c < world->manifold_map.capacity; c++) {
            if (world->manifold_map.buckets[c].occupied) {
                manifold_solve(&world->manifold_map.buckets[c].value, world->bodies);
            }
        }
    }

    // integrate all velocities
    for (uint32_t i = 0; i < world->bodies.count; i++) {
        Body* body = &world->bodies.items[i];
        body_integrate_velocities(body, dt);
    }
}

