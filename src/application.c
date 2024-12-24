#include <raylib.h>

#include "application.h"
#include "graphics.h"
#include "physics/array.h"
#include "physics/force.h"
#include "physics/body.h"
#include "physics/shape.h"
#include "physics/utils.h"
#include "physics/vec2.h"
#include "physics/collision.h"
#include "physics/contact.h"

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
static BodyArray bodies = DA_NULL;
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

    float x_center = WINDOW_WIDTH / 2.0;
    float y_center = WINDOW_HEIGHT / 2.0;
    
    Body* floor = DA_NEXT_PTR(&bodies);
    *floor = body_create_box(WINDOW_WIDTH - 50, 50, x_center, WINDOW_HEIGHT - 50, 0.0);
    floor->restitution = 0.8;
    floor->friction = 0.2;

    Body* left_wall = DA_NEXT_PTR(&bodies);
    *left_wall = body_create_box(50, WINDOW_HEIGHT - 100, 50, WINDOW_HEIGHT / 2 - 25, 0.0);
    left_wall->restitution = 0.2;
    left_wall->friction = 0.2;

    Body* right_wall = DA_NEXT_PTR(&bodies);
    *right_wall = body_create_box(50, WINDOW_HEIGHT - 100, WINDOW_WIDTH - 50, WINDOW_HEIGHT / 2 - 25, 0.0);
    right_wall->restitution = 0.2;
    right_wall->friction = 0.2;

    Body* static_box = DA_NEXT_PTR(&bodies);
    *static_box = body_create_box(300, 300, x_center, y_center, 0.0);
    static_box->rotation = 1.4;
    static_box->restitution = 0.5;
    static_box->friction = 0.2;
    body_set_texture(static_box, "./assets/crate.png");
}

void destroy() {
    for (int i = 0; i < bodies.count; i++) {
        Body* body = &bodies.items[i];
        bool is_polygon = body->shape.type == POLYGON_SHAPE || body->shape.type == BOX_SHAPE;
        if (is_polygon) {
            free(body->shape.as.polygon.local_vertices.items);
            free(body->shape.as.polygon.world_vertices.items);
        }
        if (body->texture.id) {
            UnloadTexture(body->texture);
        }
    }
    DA_FREE(&bodies); // useless because program is going to be closed (let it leak)
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
        left_mouse_down = true;
    } else if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        right_mouse_down = true;
    } else if (left_mouse_down && IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
        left_mouse_down = false;

        //circle
        Body* new_circle = DA_NEXT_PTR(&bodies);
        *new_circle = body_create_circle(30, mouse_coord.x, mouse_coord.y, 1.0);
        new_circle->restitution = 1.0f;
        new_circle->friction = 1.0f;
        body_set_texture(new_circle, "./assets/basketball.png");
    } else if (right_mouse_down && IsMouseButtonReleased(MOUSE_BUTTON_RIGHT)) {
        right_mouse_down = false;

        // box
        /*Body* new_box = DA_NEXT_PTR(&bodies);*/
        /**new_box = body_create_box(80, 80, mouse_coord.x, mouse_coord.y, 5.0);*/
        /*new_box->restitution = 0.2f;*/
        /*new_box->friction = 0.2f;*/
        /*body_set_texture(new_box, "./assets/crate.png");*/

        // polygon
        Body* new_poly = DA_NEXT_PTR(&bodies);
        Vec2Array vertices = make_regular_polygon(5, 80);
        *new_poly = body_create_polygon(vertices, mouse_coord.x, mouse_coord.y, 1.0);
        new_poly->restitution = 0.2f;
        new_poly->friction = 0.2f;
        free(vertices.items);
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
            printf("FPS: %d\n", frame_count);
            frame_count = 0;
            prev_time_fps = cur_time;
        }
    #endif

    // forces
    for (int i = 0; i < bodies.count; i++) {
        Body* body = &bodies.items[i];

        // force from arrow keys
        body_add_force(body, push_force);

        // weight
        Vec2 weight = VEC2(0.0,  (9.8 / body->inv_mass) * PIXELS_PER_METER);
        body_add_force(body, weight);

        // wind
        /*Vec2 wind = VEC2(2.0 * PIXELS_PER_METER, 0);*/
        /*body_add_force(body, wind);*/

        /*float torque = 200;*/
        /*body_add_torque(body, torque);*/

        /*Vec2 friction = force_generate_friction(body, 5 * PIXELS_PER_METER);*/
        /*body_add_force(body, friction);*/
    }

    // update body
    for (int i = 0; i < bodies.count; i++) {
        Body* body = &bodies.items[i];
        body_update(body, delta_time);
    }

    for (int i = 0; i < bodies.count; i++) {
        bodies.items[i].is_colliding = false;
    }

    // collision detection
    for (int i = 0; i < bodies.count - 1; i++) {
        for (int j = i + 1; j < bodies.count; j++) {
            Body* a = &bodies.items[i];
            Body* b = &bodies.items[j];
            Contact contact;
            if (collision_iscolliding(a, b, &contact)) {
                contact_resolve_collision(&contact);

                if (debug) {
                    // draw debug contact information
                    draw_fill_circle(contact.start.x, contact.start.y, 3, 0xFF00FFFF);
                    draw_fill_circle(contact.end.x, contact.end.y, 3, 0xFF00FFFF);
                    draw_line(contact.start.x, contact.start.y, 
                            contact.start.x + contact.normal.x * 15,
                            contact.start.y + contact.normal.y * 15,
                            0xFF00FFFF);
                    a->is_colliding = true;
                    b->is_colliding = true;
                }
            }
        }
    }
}

// TODO: map from my Vec2 type to raylib's Vector2 before rendering
/*static Vector2[] Vec2_to_Vector2(Vec2Array* array) {*/
/*}*/

void render() {
    // bodies
    for (int i = 0; i < bodies.count; i++) {
        Body* body = &bodies.items[i];
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

