#include "world.h"
#include "array.h"
#include "constraint.h"
#include "collision.h"
#include "manifold.h"
#include <raylib.h>

#define SOLVE_ITERATIONS 8

void world_init(World* world, float gravity) {
    world->gravity = gravity;
}

void world_free(World* world) {
    for (uint32_t i = 0; i < world->bodies.count; i++) {
        Body* body = &world->bodies.items[i];
        bool is_polygon = body->shape.type == POLYGON_SHAPE || body->shape.type == BOX_SHAPE;
        if (is_polygon) {
            DA_FREE(&body->shape.as.polygon.local_vertices);
            DA_FREE(&body->shape.as.polygon.world_vertices);
            DA_FREE(&body->shape.as.polygon.prev_world_vertices);
        }
    }

    /*for (uint32_t i = 0; i < world->joint_constraints.count; i++) {*/
    /*    constraint_joint_free(&world->joint_constraints.items[i]);*/
    /*}*/
    /*DA_FREE(&world->joint_constraints);*/

    DA_FREE(&world->bodies);
    DA_FREE(&world->manifolds);
    DA_FREE(&world->forces);
    DA_FREE(&world->torques);
}

Body* world_new_body(World* world) {
    return DA_NEXT_PTR(&world->bodies);
}

/*JointConstraint* world_new_joint_constraint(World* world) {*/
/*    return DA_NEXT_PTR(&world->joint_constraints);*/
/*}*/

void world_add_force(World* world, Vec2 force) {
    DA_APPEND(&world->forces, force);
}

void world_add_torque(World* world, float torque) {
    DA_APPEND(&world->torques, torque);
}

Manifold* world_manifold_next(World* world) {
    for (uint32_t i = 0; i < world->manifolds.count; i++) {
        Manifold* m = &world->manifolds.items[i];
        if (m->a_index == -1) {
            return m;
        }
    }
    return DA_NEXT_PTR(&world->manifolds);
}

Manifold* world_manifold_find(World* world, int a_index, int b_index) {
    for (uint32_t i = 0; i < world->manifolds.count; i++) {
        Manifold* m = &world->manifolds.items[i];
        if (m->a_index == a_index && m->b_index == b_index)
            return m;
    }
    return NULL;
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

    // flag all the manifolds to be expired
    for (uint32_t i = 0; i < world->manifolds.count; i++) {
        world->manifolds.items[i].expired = true;
    }

    // check collisions
    double avg_delta = 0;
    int count_finds = 0;
    for (uint32_t i = 0; i < world->bodies.count - 1; i++) {
        for (uint32_t j = i + 1; j < world->bodies.count; j++) {
            Body* a = &world->bodies.items[i];
            Body* b = &world->bodies.items[j];
            Contact contacts[2];
            uint32_t num_contacts = 0;
            if (collision_iscolliding(a, b, i, j, contacts, &num_contacts)) {
                // find if there is already an existing manifold between A and B
                double ts_start = GetTime();
                Manifold* manifold = world_manifold_find(world, i, j);
                double ts_end = GetTime();
                avg_delta += ts_end - ts_start;
                count_finds++;
                if (manifold == NULL) {
                    // create new manifold
                    manifold = world_manifold_next(world);
                    manifold_init(manifold, num_contacts, i, j);
                } 
                manifold->expired = false;
                bool persistent[2] = { false };
                if (world->warm_start) {
                    for (uint32_t c = 0; c < num_contacts; c++) {
                        persistent[c] = manifold_find_existing_contact(manifold, &contacts[c]);
                    }
                }
                for (uint32_t c = 0; c < num_contacts; c++) {
                    // contact->end is pa, contact->start is pb, normal is from A to B
                    constraint_penetration_init(
                        &manifold->constraints[c], contacts[c].a_index, contacts[c].b_index,
                        contacts[c].end, contacts[c].start, contacts[c].normal, persistent[c]);
                }
                manifold->num_contacts = num_contacts;
            }
        }
    }
    avg_delta /= count_finds;
    printf("Avg manifold find time: %fms, num finds: %d\n", (avg_delta) * 1000, count_finds);

    // delete expired manifold
    for (uint32_t i = 0; i < world->manifolds.count; i++) {
        Manifold* manifold = &world->manifolds.items[i];
        if (manifold->expired && manifold->a_index != -1) {
            // set -1 as special value for dead manifolds
            manifold->a_index = -1;
        }
    }

    // solve all constraints
    /*for (uint32_t c = 0; c < world->joint_constraints.count; c++) {*/
    /*    JointConstraint* constraint = &world->joint_constraints.items[c];*/
    /*    Body* a = &world->bodies.items[constraint->a_index];*/
    /*    Body* b = &world->bodies.items[constraint->b_index];*/
    /*    constraint_joint_pre_solve(constraint, a, b, dt);*/
    /*}*/
    for (uint32_t c = 0; c < world->manifolds.count; c++) {
        if (world->manifolds.items[c].a_index != -1) {
            Manifold* manifold = &world->manifolds.items[c];
            manifold_pre_solve(manifold, world->bodies, dt);
        }
    }
    for (uint32_t i = 0; i < SOLVE_ITERATIONS; i++) {
        // joint
        /*for (uint32_t c = 0; c < world->joint_constraints.count; c++) {*/
        /*    JointConstraint* constraint = &world->joint_constraints.items[c];*/
        /*    Body* a = &world->bodies.items[constraint->a_index];*/
        /*    Body* b = &world->bodies.items[constraint->b_index];*/
        /*    constraint_joint_solve(constraint, a, b);*/
        /*}*/
        for (uint32_t c = 0; c < world->manifolds.count; c++) {
            if (world->manifolds.items[c].a_index != -1) {
                Manifold* manifold = &world->manifolds.items[c];
                manifold_solve(manifold, world->bodies);
            }
        }
    }

    // integrate all velocities
    for (uint32_t i = 0; i < world->bodies.count; i++) {
        Body* body = &world->bodies.items[i];
        body_integrate_velocities(body, dt);
    }
}

