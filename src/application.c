#include <raylib.h>

#include "application.h"
#include "graphics.h"
#include "physics/particle.h"

#include "physics/constants.h"
#include "physics/vec2.h"

#define SHOW_FPS 0

#if SHOW_FPS

#include <stdio.h>
static int frame_count = 0;
static double prev_time_fps = 0.0;

#endif

bool running;

static Particle particle;


void setup() {
    open_window();
    running = true;
    
    init_particle(&particle, 50, 100, 1.0f, 4);
}

void destroy() {
    close_window();
}

void input() {
    if (WindowShouldClose() || IsKeyDown(KEY_ESCAPE)) {
        running = false;
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

    particle.acceleration = (Vec2) { .x = 2.0 * PIXELS_PER_METER, .y = 9.8 * PIXELS_PER_METER };
    integrate_particle(&particle, delta_time);

    // limit particle inside boundaries
    if (particle.position.y >= WINDOW_HEIGHT - particle.radius) {
        particle.position.y = WINDOW_HEIGHT - particle.radius;
        particle.velocity.y *= -1;
    }

    if (particle.position.y < particle.radius) {
        particle.position.y = particle.radius;
        particle.velocity.y *= -1;
    }

    if (particle.position.x >= WINDOW_WIDTH - particle.radius) {
        particle.position.x = WINDOW_WIDTH - particle.radius;
        particle.velocity.x *= -1;
    }

    if (particle.position.x < particle.radius) {
        particle.position.x = particle.radius;
        particle.velocity.x *= -1;
    }
}

void render() {
    begin_frame();
    clear_screen(0x056263FF);
    draw_fill_circle(particle.position.x, particle.position.y, 4, 0xFFFFFFFF);
    end_frame();
}


