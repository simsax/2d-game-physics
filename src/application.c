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
    
    init_particle(&particle, 50, 100, 1.0f);
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
    int time_to_wait = MILLISECS_PER_FRAME - (GetTime() - prev_time) * 1000;
    if (time_to_wait > 0) {
        WaitTime((double)time_to_wait / 1000);
    }

    float delta_time = GetTime() - prev_time;

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

    // integrate
    particle.velocity = (Vec2) { .x = 100, .y = 30 };
    particle.position = vec_add(particle.position, vec_scale(particle.velocity, delta_time));
}

void render() {
    begin_frame();
    clear_screen(0x056263FF);
    draw_fill_circle(particle.position.x, particle.position.y, 4, 0xFFFFFFFF);
    end_frame();
}


