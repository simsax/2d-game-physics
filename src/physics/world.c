#include "world.h"
#include "array.h"
#include "constraint.h"
#include "contact.h"
#include "collision.h"
#include "manifold.h"
#include "utils.h"
#include "../graphics.h"

World world_create(float gravity) {
    return (World) {
        .gravity = -gravity // y points down in screen space
    };
}

void world_free(World* world) {
    for (int i = 0; i < world->bodies.count; i++) {
        Body* body = &world->bodies.items[i];
        bool is_polygon = body->shape.type == POLYGON_SHAPE || body->shape.type == BOX_SHAPE;
        if (is_polygon) {
            free(body->shape.as.polygon.local_vertices.items);
            free(body->shape.as.polygon.world_vertices.items);
        }
        if (body->texture.id) {
            UnloadTexture(body->texture);
        }
    }
    for (int i = 0; i < world->joint_constraints.count; i++) {
        constraint_joint_free(&world->joint_constraints.items[i]);
    }
    for (int i = 0; i < world->manifolds.count; i++) {
        Manifold* m = &world->manifolds.items[i];
        if (m->a_index != -1) {
            manifold_free(&world->manifolds.items[i]);
        }
    }
    DA_FREE(&world->bodies);
    DA_FREE(&world->joint_constraints);
    DA_FREE(&world->manifolds);
    DA_FREE(&world->forces);
    DA_FREE(&world->torques);
}

Body* world_new_body(World* world) {
    return DA_NEXT_PTR(&world->bodies);
}

JointConstraint* world_new_joint_constraint(World* world) {
    return DA_NEXT_PTR(&world->joint_constraints);
}

void world_add_force(World* world, Vec2 force) {
    DA_APPEND(&world->forces, force);
}

void world_add_torque(World* world, float torque) {
    DA_APPEND(&world->torques, torque);
}

Manifold* world_manifold_next(World* world) {
    for (int i = 0; i < world->manifolds.count; i++) {
        Manifold* m = &world->manifolds.items[i];
        if (m->a_index == -1) {
            return m;
        }
    }
    return DA_NEXT_PTR(&world->manifolds);
}

Manifold* world_manifold_find(World* world, int a_index, int b_index) {
    for (int i = 0; i < world->manifolds.count; i++) {
        Manifold* m = &world->manifolds.items[i];
        if (m->a_index == a_index && m->b_index == b_index)
            return m;
    }
    return NULL;
}

void world_update(World* world, float dt) {
    // apply all the forces
    for (int i = 0; i < world->bodies.count; i++) {
        Body* body = &world->bodies.items[i];

        // add weight force
        Vec2 weight = VEC2(0.0,  (world->gravity / body->inv_mass) * PIXELS_PER_METER);
        body_add_force(body, weight);

        // add forces
        for (int f = 0;  f < world->forces.count; f++) {
            body_add_force(body, world->forces.items[f]);
        }

        // add torques
        for (int t = 0;  t < world->torques.count; t++) {
            body_add_torque(body, world->torques.items[t]);
        }
    }

    // integrate all the forces
    for (int i = 0; i < world->bodies.count; i++) {
        Body* body = &world->bodies.items[i];
        body_integrate_forces(body, dt);
    }

    // flag all the manifolds to be expired
    for (int i = 0; i < world->manifolds.count; i++) {
        world->manifolds.items[i].expired = true;
    }

    // check collisions
    bool warm_start = false;
    for (int i = 0; i < world->bodies.count - 1; i++) {
        for (int j = i + 1; j < world->bodies.count; j++) {
            Body* a = &world->bodies.items[i];
            Body* b = &world->bodies.items[j];
            Contact contacts[2] = { NULL_CONTACT };
            int num_contacts = 0;
            if (collision_iscolliding(a, b, contacts, &num_contacts)) {
                // find if there is already an existing manifold between A and B
                Manifold* existing_manifold = world_manifold_find(world, i, j);
                Manifold* new_manifold = NULL;
                if (existing_manifold == NULL) {
                    // create new manifold
                    new_manifold = world_manifold_next(world);
                } else {
                    existing_manifold->expired = false;
                    if (!warm_start || !manifold_contact_almost_equal(existing_manifold, contacts, num_contacts)) {
                        // overwrite existing manifold
                        manifold_free(existing_manifold);
                        new_manifold = existing_manifold;
                    }
                    // else, do nothing, we re-use the manifold in the next frame
                    // TODO: add sleep
                }
                if (new_manifold != NULL) {
                    *new_manifold = manifold_create(num_contacts, i, j);
                    for (int c = 0; c < num_contacts; c++) {
                        // draw collision points
                        draw_fill_circle(contacts[c].start.x, contacts[c].start.y, 4, 0xFF0000FF);
                        draw_fill_circle(contacts[c].end.x, contacts[c].end.y, 2, 0xFF0000FF);
                        // draw normal
                        Vec2 end_normal = vec2_add(contacts[c].start, vec2_mult(contacts[c].normal, 16));
                        draw_line(contacts[c].start.x, contacts[c].start.y, end_normal.x, end_normal.y, 0x00FF00FF);
                        
                        // create new penetration constraint
                        new_manifold->constraints[c] = constraint_penetration_create(
                                world, i, j, contacts[c].start, contacts[c].end, contacts[c].normal);
                    }
                }
            }
        }
    }

    // delete expired manifold
    for (int i = 0; i < world->manifolds.count; i++) {
        Manifold* manifold = &world->manifolds.items[i];
        if (manifold->expired && manifold->a_index != -1) {
            manifold_free(manifold);
            // set -1 as special value for dead manifolds
            manifold->a_index = -1;
        }
    }

    // solve all constraints
    for (int c = 0; c < world->joint_constraints.count; c++) {
        constraint_joint_pre_solve(&world->joint_constraints.items[c], dt);
    }
    for (int c = 0; c < world->manifolds.count; c++) {
        if (world->manifolds.items[c].a_index != -1)
            manifold_pre_solve(&world->manifolds.items[c], dt);
    }
    for (int i = 0; i < 5; i++) {
        for (int c = 0; c < world->joint_constraints.count; c++) {
            constraint_joint_solve(&world->joint_constraints.items[c]);
        }
        for (int c = 0; c < world->manifolds.count; c++) {
            if (world->manifolds.items[c].a_index != -1)
                manifold_solve(&world->manifolds.items[c]);
        }
    }
    for (int c = 0; c < world->joint_constraints.count; c++) {
        constraint_joint_post_solve(&world->joint_constraints.items[c]);
    }
    for (int c = 0; c < world->manifolds.count; c++) {
        if (world->manifolds.items[c].a_index != -1)
            manifold_post_solve(&world->manifolds.items[c]);
    }

    // integrate all velocities
    for (int i = 0; i < world->bodies.count; i++) {
        Body* body = &world->bodies.items[i];
        body_integrate_velocities(body, dt);
    }
}

