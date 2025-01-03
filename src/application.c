#include <raylib.h>

#include "application.h"
#include "graphics.h"
#include "physics/array.h"
#include "physics/constraint.h"
#include "physics/force.h"
#include "physics/body.h"
#include "physics/shape.h"
#include "physics/utils.h"
#include "physics/vec2.h"
#include "physics/collision.h"
#include "physics/contact.h"
#include "physics/world.h"

#include <stdint.h>
#include <stdio.h>

#define SHOW_FPS 1

#if SHOW_FPS

static int frame_count = 0;
static double prev_time_fps = 0.0;

#endif

// public globals
bool running;
static bool debug = false;

// private globals
static World world;
static Vec2 push_force = VEC2(0, 0);
static bool left_mouse_down = false;
static bool right_mouse_down = false;
static Vec2 mouse_coord = VEC2(0, 0);

// ambitious TODO: take all commits, turn them into a full 2d physics demo project with different scenes
// curate all of them (for ex. gravitation add textures, planets and stars)

static Vec2Array make_regular_polygon(int num_vertices, int radius) {
    Vec2Array vertices = DA_NULL;

    float angle_offset = 2.0f * PI / num_vertices;
    for (int i = 0; i < num_vertices; i++) {
        float angle = i * angle_offset;
        DA_APPEND(&vertices, VEC2(cos(angle) * radius, sin(angle) * radius));
    }

    return vertices;
}

void setup() {
    open_window();
    running = true;

    world = world_create(-9.8);

    float x_center = WINDOW_WIDTH / 2.0;
    float y_center = WINDOW_HEIGHT / 2.0;
    
    int num_bodies = 8;
    for (int i = 0; i < num_bodies; i++) {
        float mass = (i == 0) ? 0.0f : 1.0f;
        Body* body = world_new_body(&world);
        *body = body_create_box(30, 30, x_center - i * 40.0f, 100, mass);
        body_set_texture(body, "./assets/crate.png");
    }

    for (int i = 0; i < num_bodies - 1; i++) {
        Body* a = &world.bodies.items[i];
        Body* b = &world.bodies.items[i + 1];
        Constraint* c = world_new_constraint(&world);
        *c = constraint_create(JOINT_CONSTRAINT, &world, i, i + 1, a->position);
    }
}

void destroy() {
    world_free(&world);
    close_window();
}

void input() {
    if (WindowShouldClose() || IsKeyDown(KEY_ESCAPE)) {
        running = false;
    }

    const float force = 50 * PIXELS_PER_METER;
    if (IsKeyDown(KEY_UP)) {
        push_force.y = -force;
    }
    if (IsKeyDown(KEY_DOWN)) {
        push_force.y = force;
    }
    if (IsKeyDown(KEY_LEFT)) {
        push_force.x = -force;
    }
    if (IsKeyDown(KEY_RIGHT)) {
        push_force.x = force;
    }
    if (IsKeyReleased(KEY_UP)) {
        push_force.y = 0;
    }
    if (IsKeyReleased(KEY_DOWN)) {
        push_force.y = 0;
    }
    if (IsKeyReleased(KEY_LEFT)) {
        push_force.x = 0;
    }
    if (IsKeyReleased(KEY_RIGHT)) {
        push_force.x = 0;
    }
    if (IsKeyPressed(KEY_D)) {
        debug = !debug;
    }

    // mouse
    mouse_coord.x = GetMouseX();
    mouse_coord.y = GetMouseY();
    
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        //circle
        Body* new_circle = world_new_body(&world);
        *new_circle = body_create_circle(30, mouse_coord.x, mouse_coord.y, 1.0);
        new_circle->restitution = 1.0f;
        new_circle->friction = 1.0f;
        body_set_texture(new_circle, "./assets/basketball.png");
    } else if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        // box
        Body* new_box = world_new_body(&world);
        *new_box = body_create_box(80, 80, mouse_coord.x, mouse_coord.y, 5.0);
        new_box->restitution = 0.2f;
        new_box->friction = 0.2f;
        body_set_texture(new_box, "./assets/crate.png");

        // polygon
        /*Body* new_poly = world_new_body(&world);*/
        /*Vec2Array vertices = make_regular_polygon(5, 80);*/
        /**new_poly = body_create_polygon(vertices, mouse_coord.x, mouse_coord.y, 1.0);*/
        /*new_poly->restitution = 0.2f;*/
        /*new_poly->friction = 0.2f;*/
        /*free(vertices.items);*/
    }
}


void update() {
    // for debug draws
    begin_frame();
    clear_screen(0x056263FF);

    static float prev_time = 0;
    // wait until target frame time is reached
    float delta_time_prev_frame = GetTime() - prev_time;
    float time_to_wait = SECS_PER_FRAME - delta_time_prev_frame;
    if (time_to_wait > 0) {
        WaitTime(time_to_wait);
    }

    // new frame begins
    float cur_time = GetTime();
    float delta_time = cur_time - prev_time;
    if (delta_time > SECS_PER_FRAME) {
        // cap delta_time so that this thing is debuggable
        delta_time = SECS_PER_FRAME;
    }
    prev_time = cur_time;

    #if SHOW_FPS
        frame_count++;
        if (cur_time - prev_time_fps >= 1.0f) {
            printf("FPS: %d | Num objects: %d\n", frame_count, world.bodies.count);
            frame_count = 0;
            prev_time_fps = cur_time;
        }
    #endif

    world_update(&world, delta_time);
}

// TODO: map from my Vec2 type to raylib's Vector2 before rendering
/*static Vector2[] Vec2_to_Vector2(Vec2Array* array) {*/
/*}*/

void render() {
    // joints
    for (int i = 0; i < world.constraints.count; i++) {
        Constraint* constraint = &world.constraints.items[i];
        Body* a = &world.bodies.items[constraint->a_index];
        Body* b = &world.bodies.items[constraint->b_index];
        if (constraint->type == JOINT_CONSTRAINT) {
            Vec2 pa = body_local_to_world_space(a, constraint->a_point);
            Vec2 pb = body_local_to_world_space(b, constraint->a_point);
            draw_line(pa.x, pa.y, pb.x, pb.y, 0xFFFFFFFF);
        }
    }

    // bodies
    for (int i = 0; i < world.bodies.count; i++) {
        Body* body = &world.bodies.items[i];
        uint32_t color = 0xFFFFFFFF;
        if (body->shape.type == CIRCLE_SHAPE) {
            if (!debug && body->texture.id) {
                float diameter = body->shape.as.circle.radius * 2;
                draw_texture(body->position.x, body->position.y, diameter, diameter,
                        body->rotation, &body->texture);
            } else {
                draw_circle_line(body->position.x, body->position.y,
                        body->shape.as.circle.radius, body->rotation, color);
            }
        }  
        if (body->shape.type == BOX_SHAPE) {
            BoxShape* box_shape = &body->shape.as.box;
            if (!debug && body->texture.id) {
                draw_texture(body->position.x, body->position.y, box_shape->width,
                        box_shape->height, body->rotation, &body->texture);
            } else {
                draw_polygon(body->position.x, body->position.y, box_shape->polygon.world_vertices, color);
            }
        }
        if (body->shape.type == POLYGON_SHAPE) {
            PolygonShape* polygon_shape = &body->shape.as.polygon;
            if (!debug) {
                draw_fill_polygon(body->position.x, body->position.y, polygon_shape->world_vertices, color);
            } else {
                draw_polygon(body->position.x, body->position.y, polygon_shape->world_vertices, color);
            }
        }
    }

    end_frame();
}

