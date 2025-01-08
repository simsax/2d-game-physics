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
        manifold_free(&world->manifolds.items[i]);
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

Manifold* world_new_manifold(World* world) {
    return DA_NEXT_PTR(&world->manifolds);
}

void world_add_force(World* world, Vec2 force) {
    DA_APPEND(&world->forces, force);
}

void world_add_torque(World* world, float torque) {
    DA_APPEND(&world->torques, torque);
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

    // check collisions
    ManifoldArray new_manifolds = DA_NULL;
    for (int i = 0; i < world->bodies.count - 1; i++) {
        for (int j = i + 1; j < world->bodies.count; j++) {
            Body* a = &world->bodies.items[i];
            Body* b = &world->bodies.items[j];
            Contact contacts[2] = { NULL_CONTACT };
            int num_contacts = 0;
            if (collision_iscolliding(a, b, contacts, &num_contacts)) {
                // Dumb implementation:
                // 1. initialize new manifold array
                // 2. for each new contact, check if it already exists (distance smaller than given threshold)
                // 3. if it exists, then reuse the old one and copy it into new array (warm starting)
                // 4. if new, insert it into the array
                // 5. new array should contain both old constraints and new ones, so expired ones are automatically deleted
                Manifold* new_manifold = DA_NEXT_PTR(&new_manifolds);
                bool contact_is_new = true;
                for (int k = 0; k < world->manifolds.count; k++) {
                    Manifold* manifold = &world->manifolds.items[k];
                    if (manifold_contact_almost_equal(manifold, contacts, num_contacts)) {
                        // copy old manifold, slow but for now it's fine
                        // TODO: change implementation
                        *new_manifold = *manifold;
                        contact_is_new = false;
                        break;
                    }
                }
                if (contact_is_new) {
                    *new_manifold = manifold_create(num_contacts);
                    for (int c = 0; c < num_contacts; c++) {
                        /*// draw collision points*/
                        /*draw_fill_circle(contacts[c].start.x, contacts[c].start.y, 5, 0xFF0000FF);*/
                        /*draw_fill_circle(contacts[c].end.x, contacts[c].end.y, 2, 0xFF0000FF);*/
                        
                        // create new penetration constraint
                        new_manifold->constraints[c] = constraint_penetration_create(
                                world, i, j, contacts[c].start, contacts[c].end, contacts[c].normal);
                    }
                }
            }
        }
    }
    // free old manifolds, set new_manifolds as world manifolds
    // TODO: this code leaks memory...
    // cannot free because I'd lose the data inside PenetrationConstraints...
    /*for (int i = 0; i < world->manifolds.count; i++) {*/
    /*    manifold_free(&world->manifolds.items[i]);*/
    /*}*/
    /*DA_FREE(&world->manifolds);*/
    world->manifolds = new_manifolds;

    // solve all constraints
    for (int c = 0; c < world->joint_constraints.count; c++) {
        constraint_joint_pre_solve(&world->joint_constraints.items[c], dt);
    }
    for (int c = 0; c < world->manifolds.count; c++) {
        manifold_pre_solve(&world->manifolds.items[c], dt);
    }
    for (int i = 0; i < 5; i++) {
        for (int c = 0; c < world->joint_constraints.count; c++) {
            constraint_joint_solve(&world->joint_constraints.items[c]);
        }
        for (int c = 0; c < world->manifolds.count; c++) {
            manifold_solve(&world->manifolds.items[c]);
        }
    }
    for (int c = 0; c < world->joint_constraints.count; c++) {
        constraint_joint_post_solve(&world->joint_constraints.items[c]);
    }
    for (int c = 0; c < world->manifolds.count; c++) {
        manifold_post_solve(&world->manifolds.items[c]);
    }

    // integrate all velocities
    for (int i = 0; i < world->bodies.count; i++) {
        Body* body = &world->bodies.items[i];
        body_integrate_velocities(body, dt);
    }
}

