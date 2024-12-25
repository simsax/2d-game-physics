#include "world.h"
#include "array.h"
#include "contact.h"
#include "collision.h"
#include "utils.h"

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
    DA_FREE(&world->bodies);
    DA_FREE(&world->forces);
    DA_FREE(&world->torques);
}

Body* world_new_body(World* world) {
    return DA_NEXT_PTR(&world->bodies);
}

void world_add_force(World* world, Vec2 force) {
    DA_APPEND(&world->forces, force);
}

void world_add_torque(World* world, float torque) {
    DA_APPEND(&world->torques, torque);
}

void world_update(World* world, float dt) {
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

        // integrate forces to find new position and rotation
        body_update(body, dt);

        // reset debug information
        body->is_colliding = false;
    }

    // apply collision detection and resolution for all bodies in the world
    for (int n = 0; n < 10; n++)
        world_check_collisions(world);
}

void world_check_collisions(World* world) {
    for (int i = 0; i < world->bodies.count - 1; i++) {
        for (int j = i + 1; j < world->bodies.count; j++) {
            Body* a = &world->bodies.items[i];
            Body* b = &world->bodies.items[j];
            Contact contact;
            if (collision_iscolliding(a, b, &contact)) {
                contact_resolve_collision(&contact);
            }
        }
    }
}

