#include <raylib.h>

#include "application.h"
#include "graphics.h"
#include "physics/force.h"
#include "physics/body.h"
#include "physics/shape.h"
#include "physics/utils.h"
#include "physics/vec2.h"

#include <stdio.h>

#define SHOW_FPS 1

#if SHOW_FPS

static int frame_count = 0;
static double prev_time_fps = 0.0;

#endif

// public globals
bool running;

// private globals
static BodyArray bodies = DA_NULL;
static Vec2 push_force = VEC2(0, 0);
static bool left_mouse_down = false;
static Vec2 mouse_coord = VEC2(0, 0);

// ambitious TODO: take all commits, turn them into a full 2d physics demo project with different scenes
// curate all of them (for ex. gravitation add textures, planets and stars)
// then post it in the raylib subreddit (give credit to pikuma as well)

void setup() {
    open_window();
    running = true;

    float x_center = WINDOW_WIDTH / 2.0;
    float y_center = WINDOW_HEIGHT / 2.0;
    
    DA_APPEND(&bodies, body_create_box(200, 100, x_center, y_center, 1.0));
}

void destroy() {
    for (int i = 0; i < bodies.count; i++) {
        Body* body = &bodies.items[i];
        bool is_polygon = body->shape.type == POLYGON_SHAPE || body->shape.type == BOX_SHAPE;
        if (is_polygon) {
            free(body->shape.as.polygon.local_vertices.items);
            free(body->shape.as.polygon.world_vertices.items);
        }
    }
    DA_FREE(&bodies); // useless because program is going to be closed (it's fine to leak memory if it's not in a loop)
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

    // mouse
    mouse_coord.x = GetMouseX();
    mouse_coord.y = GetMouseY();
    
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        left_mouse_down = true;
    } else if (left_mouse_down && IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
        left_mouse_down = false;
        /*Vec2 impulse_direction = vec_sub(bodies.items[0].position, mouse_coord);*/
        /*float impulse_magnitude = vec_magnitude(impulse_direction) * 5;*/
        /*bodies.items[0].velocity = vec_mult(vec_normalize(impulse_direction), impulse_magnitude);*/
    }
}

static void constrain_euler() {
    // limit bodies inside window boundaries
    for (int i = 0; i < bodies.count; i++) {
        Body* body = &bodies.items[i];

        if (body->shape.type == CIRCLE_SHAPE) {
            CircleShape* circle_shape = &body->shape.as.circle;
            if (body->position.y >= WINDOW_HEIGHT - circle_shape->radius) {
                body->position.y = WINDOW_HEIGHT - circle_shape->radius;
                body->velocity.y *= -1;
            }

            if (body->position.y < circle_shape->radius) {
                body->position.y = circle_shape->radius;
                body->velocity.y *= -1;
            }

            if (body->position.x >= WINDOW_WIDTH - circle_shape->radius) {
                body->position.x = WINDOW_WIDTH - circle_shape->radius;
                body->velocity.x *= -1;
            }

            if (body->position.x < circle_shape->radius) {
                body->position.x = circle_shape->radius;
                body->velocity.x *= -1;
            }
        }
    }
}

void update() {
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
        /*Vec2 weight = VEC2(0.0,  (9.8 / body->inv_mass) * PIXELS_PER_METER);*/
        /*body_add_force(body, weight);*/


        float torque = 200;
        body_add_torque(body, torque);

        /*Vec2 friction = force_generate_friction(body, 5 * PIXELS_PER_METER);*/
        /*body_add_force(body, friction);*/

    }

    // update body
    for (int i = 0; i < bodies.count; i++) {
        Body* body = &bodies.items[i];
        body_update(body, delta_time);
    }

    constrain_euler();
}

void render() {
    begin_frame();
    clear_screen(0x056263FF);

    // bodies
    for (int i = 0; i < bodies.count; i++) {
        Body* body = &bodies.items[i];
        if (body->shape.type == CIRCLE_SHAPE) {
            draw_circle_line(body->position.x, body->position.y,
                    body->shape.as.circle.radius, body->rotation, 0xEEEEEEFF);
        }  
        if (body->shape.type == BOX_SHAPE) {
            BoxShape* box_shape = &body->shape.as.box;
            draw_polygon(body->position.x, body->position.y, &box_shape->polygon.world_vertices, 0xFFFFFFFF);
        }
    }

    end_frame();
}

