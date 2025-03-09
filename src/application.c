#include <raylib.h>

#include "application.h"
#include "graphics.h"
#include "physics/constraint.h"
#include "physics/body.h"
#include "physics/shape.h"
#include "physics/utils.h"
#include "physics/vec2.h"
#include "physics/world.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define SHOW_FPS 1

#if SHOW_FPS

static int frame_count = 0;
static float prev_time_fps = 0.0f;

#endif

// palette https://coolors.co/1a0f0d-392426-6b2c2e-925d5e-5d1816-251a1a-150705
#define COLOR_BACKGROUND 0x282828FF
#define COLOR_CIRCLE 0xB8BB26FF
#define COLOR_BOX 0xD3869BFF

// starts tanking at 900 boxes
// 30 fps with 1200 boxes
// 12 fps with 2000 boxes

// public globals
bool running;
static bool paused = false;
static bool debug = true;

// private globals
static World world;
/*static Arena arena;*/
static Vec2 mouse_coord = {0, 0};


/*static Vec2Array make_regular_polygon(int num_vertices, int radius) {*/
/*    Vec2Array vertices = DA_NULL;*/
/**/
/*    float angle_offset = 2.0f * PI / num_vertices;*/
/*    for (int i = 0; i < num_vertices; i++) {*/
/*        float angle = i * angle_offset;*/
/*        DA_APPEND(&vertices, VEC2(cos(angle) * radius, sin(angle) * radius));*/
/*    }*/
/**/
/*    return vertices;*/
/*}*/
/**/

static void demo_incline_plane(void) {
    float x_center = WINDOW_WIDTH / 2.0;
    float y_center = WINDOW_HEIGHT / 2.0;
    Body* static_box = world_new_body(&world);
    body_init_box_pixels(static_box, 200, 800, x_center, y_center, 0.0);
    static_box->restitution = 0.8;
    static_box->friction = 0.2;
    static_box->rotation = 1.4;
}

static void demo_stack(void) {
    float x_center = WINDOW_WIDTH / 2.0;
    float ground = WINDOW_HEIGHT - 75.0f;
    float side_len = 80.0f;
    for (int i = 0; i < 10; i++) {
        Body* box = world_new_body(&world);
        body_init_box_pixels(box, side_len, side_len, x_center, ground - side_len / 2.0f - i * side_len, 1.0);
        box->restitution = 0.0;
        box->friction = 0.2;
    }
}

static void demo_pyramid(void) {
    int len_base = 12;
    float x_center = WINDOW_WIDTH / 2.0;
    float ground = WINDOW_HEIGHT - 75.0f;
    float side_len = 40.0f;
    float x_start = x_center - (len_base / 2.0f) * side_len;
    float y_offset = side_len * 0.25f;
    float x_offset = side_len * 1.125f;
    float y_start = ground - side_len / 2.0f - y_offset;
    for (int i = 0; i < len_base; i++) {
        float y = y_start - i * (x_offset + y_offset);
        float x_row = x_start + i * x_offset / 2.0f;
        for (int j = i; j < len_base; j++) {
            float x = x_row + (j - i) * x_offset;
            Body* box = world_new_body(&world);
            body_init_box_pixels(box, side_len, side_len, x, y, 1.0);
            box->restitution = 0.0;
            box->friction = 0.4;
        }
    }

}

// TODO: some demos work better with some values, check box2d
// ex, stack of boxes better when using more penetration slop

// values to check:
// - fraction of lambda impulse to re-use (probably just use all)
// - bias
// - penetration slop
// - restitution? For now set it 0

static void start_simulation(void) {
    world.gravity = 9.8f; // y points down in screen space

    float x_center = WINDOW_WIDTH / 2.0;
    float y_center = WINDOW_HEIGHT / 2.0;

    Body* floor = world_new_body(&world);
    body_init_box_pixels(floor, WINDOW_WIDTH - 50, 50, x_center, WINDOW_HEIGHT - 50, 0.0f);
    floor->restitution = 0.8;
    floor->friction = 0.8;

    Body* left_wall = world_new_body(&world);
    body_init_box_pixels(left_wall, 50, WINDOW_HEIGHT - 150, 50, WINDOW_HEIGHT / 2, 0.0f);
    left_wall->restitution = 0.8;
    left_wall->friction = 0.2;

    Body* right_wall = world_new_body(&world); 
    body_init_box_pixels(right_wall, 50, WINDOW_HEIGHT - 150, WINDOW_WIDTH - 50, WINDOW_HEIGHT / 2, 0.0f);
    right_wall->restitution = 0.8;
    right_wall->friction = 0.2;

    Body* ceiling = world_new_body(&world);
    body_init_box_pixels(ceiling, WINDOW_WIDTH - 50, 50, x_center, 50, 0.0f);
    ceiling->restitution = 0.8;
    ceiling->friction = 0.2;

    demo_pyramid();
}

void setup(void) {
    srand(time(NULL));
    open_window();
    running = true;
    start_simulation();
}

void destroy(void) {
    world_free(&world);
    close_window();
}

void input(void) {
    if (WindowShouldClose() || IsKeyDown(KEY_ESCAPE)) {
        running = false;
    }

    /*const float force = 50 * PIXELS_PER_METER;*/
    if (IsKeyPressed(KEY_D)) {
        debug = !debug;
    }
    if (IsKeyPressed(KEY_P)) {
        paused = !paused;
    }
    if (IsKeyPressed(KEY_R)) {
        paused = false;
        world_free(&world);
        start_simulation();
    }

    /*world.bodies.items[5].position = mouse_coord;*/

    if (!paused) {
        // mouse
        mouse_coord.x = GetMouseX();
        mouse_coord.y = GetMouseY();
        
        /*if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {*/
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            //circle
            Body* new_circle = world_new_body(&world);
            body_init_circle_pixels(new_circle, 40, mouse_coord.x, mouse_coord.y, 1.0);
            new_circle->restitution = 0.5f;
            new_circle->friction = 1.0f;
            /*body_set_texture(new_circle, "./assets/basketball.png");*/
        /*} else if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {*/
        } else if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
            // box
            Body* new_box = world_new_body(&world);
            body_init_box_pixels(new_box, 60, 60, mouse_coord.x, mouse_coord.y, 10.0);
            new_box->restitution = 0.2f;
            new_box->friction = 0.8f;

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

void update(void) {
    begin_frame();
    // TODO: fix timestep

    static float prev_time = 0;
    // wait until target frame time is reached
    float delta_time_prev_frame = (float)GetTime() - prev_time;
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
        clear_screen(COLOR_BACKGROUND);
        world_update(&world, delta_time);
    }
}

// TODO: map from my Vec2 type to raylib's Vector2 before rendering
/*static Vector2[] Vec2_to_Vector2(Vec2Array* array) {*/
/*}*/

void render(void) {
    if (!paused) {
        // bodies
        for (uint32_t i = 0; i < world.bodies.count; i++) {
            Body* body = &world.bodies.items[i];
            if (body->shape.type == CIRCLE_SHAPE) {
                /*if (!debug && body->texture.id) {*/
                /*    float diameter = body->shape.as.circle.radius * 2;*/
                /*    draw_texture(body->position.x, body->position.y, diameter, diameter,*/
                /*            body->rotation, &body->texture);*/
                /*} else {*/
                draw_circle_line_meters(body->position.x, body->position.y,
                        body->shape.as.circle.radius, body->rotation, COLOR_CIRCLE);
                /*}*/
            }  
            if (body->shape.type == BOX_SHAPE) {
                BoxShape* box_shape = &body->shape.as.box;
                /*if (!debug && body->texture.id) {*/
                /*    draw_texture(body->position.x, body->position.y, box_shape->width,*/
                /*            box_shape->height, body->rotation, &body->texture);*/
                /*} else {*/
                draw_polygon_meters(body->position.x, body->position.y, box_shape->polygon.world_vertices, COLOR_BOX);
                /*}*/
            }
            if (body->shape.type == POLYGON_SHAPE) {
                PolygonShape* polygon_shape = &body->shape.as.polygon;
                if (!debug) {
                    draw_fill_polygon_meters(body->position.x, body->position.y, polygon_shape->world_vertices, COLOR_BOX);
                } else {
                    draw_polygon_meters(body->position.x, body->position.y, polygon_shape->world_vertices, COLOR_BOX);
                }
            }
        }

        // joints
        if (debug) {
            for (uint32_t i = 0; i < world.joint_constraints.count; i++) {
                JointConstraint* constraint = &world.joint_constraints.items[i];
                Body* a = &world.bodies.items[constraint->a_index];
                Vec2 anchor = body_local_to_world_space(a, constraint->a_point);
                draw_fill_circle_meters(anchor.x, anchor.y, 3, 0xFF0000FF);
            }
        }

    }
    end_frame();
}

