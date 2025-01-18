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

// starts tanking at 900 boxes
// 30 fps with 1200 boxes
// 12 fps with 2000 boxes

// public globals
bool running;
static bool paused = false;
static bool debug = true;

// private globals
static World world;
static Vec2 push_force = VEC2(0, 0);
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

    Body* floor = world_new_body(&world);
    *floor = body_create_box(WINDOW_WIDTH - 50, 50, x_center, WINDOW_HEIGHT - 50, 0.0);
    floor->restitution = 0.8;
    floor->friction = 0.2;

    Body* left_wall = world_new_body(&world);
    *left_wall = body_create_box(50, WINDOW_HEIGHT - 150, 50, WINDOW_HEIGHT / 2, 0.0);
    left_wall->restitution = 0.8;
    left_wall->friction = 0.2;

    Body* right_wall = world_new_body(&world); 
    *right_wall = body_create_box(50, WINDOW_HEIGHT - 150, WINDOW_WIDTH - 50, WINDOW_HEIGHT / 2, 0.0);
    right_wall->restitution = 0.8;
    right_wall->friction = 0.2;

    Body* ceiling = world_new_body(&world);
    *ceiling = body_create_box(WINDOW_WIDTH - 50, 50, x_center, 50, 0.0);
    ceiling->restitution = 0.8;
    ceiling->friction = 0.2;

    Body* static_box = world_new_body(&world);
    *static_box = body_create_box(50, 800, x_center, y_center, 0.0);
    static_box->rotation = 1.4;
    static_box->restitution = 0.5;
    static_box->friction = 0.3;
    static_box->angular_velocity = 2.0f;
    shape_update_vertices(&static_box->shape, static_box->rotation, static_box->position);

    /*Body* static_ball = world_new_body(&world);*/
    /**static_ball = body_create_circle(100, x_center + 400, y_center, 0.0);*/
    /**/
    /*Body* box = world_new_body(&world);*/
    /**box = body_create_box(200, 200, x_center, y_center, 0.0);*/

    /*Body* c = world_new_body(&world);*/
    /**c = body_create_circle(100, x_center, y_center, 0.0);*/
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
    if (IsKeyPressed(KEY_P)) {
        paused = !paused;
    }

    /*world.bodies.items[5].position = mouse_coord;*/

    if (!paused) {
        // mouse
        mouse_coord.x = GetMouseX();
        mouse_coord.y = GetMouseY();
        
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        /*if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {*/
            //circle
            Body* new_circle = world_new_body(&world);
            *new_circle = body_create_circle(15, mouse_coord.x, mouse_coord.y, 1.0);
            new_circle->restitution = 0.9f;
            new_circle->friction = 1.0f;
            /*body_set_texture(new_circle, "./assets/basketball.png");*/
        } else if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        /*} else if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {*/
            // box

            // add 10 boxes (to debug)
            for (int i = 0; i < 10; i++) {
                Body* new_box = world_new_body(&world);
                *new_box = body_create_box(20, 20, mouse_coord.x, mouse_coord.y, 1.0);
                new_box->restitution = 0.2f;
                new_box->friction = 0.8f;
            }
            /*body_set_texture(new_box, "./assets/crate.png");*/

            // polygon
            /*Body* new_poly = world_new_body(&world);*/
            /*Vec2Array vertices = make_regular_polygon(5, 80);*/
            /**new_poly = body_create_polygon(vertices, mouse_coord.x, mouse_coord.y, 1.0);*/
            /*new_poly->restitution = 0.2f;*/
            /*new_poly->friction = 0.2f;*/
            /*free(vertices.items);*/
        }
    }
}

// TODO: zoom in and out with mouse wheel, ability to navigate the world
// this should be outside the scope of the physics engine though

void update() {
    begin_frame();
    // TODO: fix timestep

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

    if (!paused) {
        // for debug draws
        clear_screen(0x056263FF);
        world_update(&world, delta_time);
    }
}

// TODO: map from my Vec2 type to raylib's Vector2 before rendering
/*static Vector2[] Vec2_to_Vector2(Vec2Array* array) {*/
/*}*/

void render(void) {
    if (!paused) {
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

        // joints
        if (debug) {
            for (int i = 0; i < world.joint_constraints.count; i++) {
                JointConstraint* constraint = &world.joint_constraints.items[i];
                Body* a = &world.bodies.items[constraint->a_index];
                Vec2 anchor = body_local_to_world_space(a, constraint->a_point);
                draw_fill_circle(anchor.x, anchor.y, 3, 0xFF0000FF);
            }
        }

    }
    end_frame();
}

