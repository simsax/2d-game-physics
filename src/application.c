#include <raylib.h>

#include "application.h"
#include "graphics.h"
#include "physics/force.h"
#include "physics/particle.h"

#include "physics/constants.h"
#include "physics/vec2.h"

#include <stdio.h>

#define SHOW_FPS 0

#if SHOW_FPS

static int frame_count = 0;
static double prev_time_fps = 0.0;

#endif

// public globals
bool running;

// private globals
static ParticleArray particles = DA_NULL;
static Vec2 push_force = VEC2(0, 0);
static Rectangle liquid;

void setup() {
    open_window();
    running = true;

    liquid.x = 0;
    liquid.y = WINDOW_HEIGHT / 2.0;
    liquid.height = WINDOW_HEIGHT / 2.0;
    liquid.width = WINDOW_WIDTH;
}

void destroy() {
    DA_FREE(&particles); // useless because program is going to be closed (it's fine to leak memory if it's not in a loop)
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
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        DA_APPEND(&particles, particle_create(GetMouseX(), GetMouseY(), 1.0f, 4)); 
    }
}

void update() {
    static double prev_time = 0;
    // wait until target frame time is reached
    int time_to_wait = MILLISECS_PER_FRAME - (GetTime() - prev_time) * 1000;
    if (time_to_wait > 0) {
        WaitTime((double)time_to_wait / 1000);
    }

    float delta_time = GetTime() - prev_time;
    if (delta_time > MILLISECS_PER_FRAME) {
        // cap delta_time so that this thing is debuggable
        delta_time = MILLISECS_PER_FRAME;
    }

    #if SHOW_FPS
        frame_count++;
        double cur_time = GetTime();
        if (cur_time - prev_time_fps >= 1.0f) {
            printf("FPS: %d\n", frame_count);
            frame_count = 0;
            prev_time_fps = cur_time;
        }
    #endif

    prev_time = GetTime();


    // forces
    for (int i = 0; i < particles.count; i++) {
        Particle* particle = &particles.items[i];

        Vec2 wind = VEC2(0.4 * PIXELS_PER_METER, 0);
        Vec2 weight = VEC2(0, particle->mass * 9.8 * PIXELS_PER_METER);

        particle_add_force(particle, weight);
        particle_add_force(particle, push_force);

        // apply drag if we are inside the liquid
        if (particle->position.y >= liquid.y) {
            Vec2 drag = force_generate_drag(particle, 0.04);
            particle_add_force(particle, drag);
        }
    }

    for (int i = 0; i < particles.count; i++) {
        Particle* particle = &particles.items[i];

        particle_integrate(particle, delta_time);
    }

    for (int i = 0; i < particles.count; i++) {
        Particle* particle = &particles.items[i];

        // limit particle inside boundaries
        if (particle->position.y >= WINDOW_HEIGHT - particle->radius) {
            particle->position.y = WINDOW_HEIGHT - particle->radius;
            particle->velocity.y *= -1;
        }

        if (particle->position.y < particle->radius) {
            particle->position.y = particle->radius;
            particle->velocity.y *= -1;
        }

        if (particle->position.x >= WINDOW_WIDTH - particle->radius) {
            particle->position.x = WINDOW_WIDTH - particle->radius;
            particle->velocity.x *= -1;
        }

        if (particle->position.x < particle->radius) {
            particle->position.x = particle->radius;
            particle->velocity.x *= -1;
        }
    }
}

void render() {
    begin_frame();
    clear_screen(0x056263FF);

    // draw liquid
    draw_fill_rect(liquid.x, liquid.y, liquid.width, liquid.height, 0x13376EFF);

    // draw particles
    for (int i = 0; i < particles.count; i++) {
        Particle* particle = &particles.items[i];
        draw_fill_circle(particle->position.x, particle->position.y, particle->radius, 0xFFFFFFFF);
    }

    end_frame();
}


