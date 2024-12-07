#include <raylib.h>

#include "application.h"
#include "graphics.h"
#include "physics/force.h"
#include "physics/particle.h"

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
static ParticleArray particles = DA_NULL;
static Vec2 push_force = VEC2(0, 0);
static Rectangle liquid;
static bool left_mouse_down = false;
static Vec2 mouse_coord = VEC2(0, 0);
static Vec2 anchor = VEC2(WINDOW_WIDTH / 2.0, 30);
static float k = 500;
static float rest_length = 80;
static int particle_radius = 10;
static int hover_index = -1;
static int hover_radius;
static int selected_index = -1;

// TODO: select closest with mouse, try a 2d plane of particles

void setup() {
    open_window();
    running = true;

    float x_coord = WINDOW_WIDTH / 2.0;
    float particle_offset = 80.0f;
    float y_start = 120.0f;

    int num_particles = 10;

    for (int i = 0; i < num_particles; i++) {
        DA_APPEND(&particles, particle_create(x_coord, y_start + particle_offset * i, 2.0, particle_radius)); 
    }
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
    mouse_coord.x = GetMouseX();
    mouse_coord.y = GetMouseY();
    
    // check if hovering on a particle
    hover_radius = fmax(particle_radius, 14);
    if (selected_index == -1) {
        for (int i = 0; i < particles.count; i++) {
            Particle* particle = &particles.items[i];
            
            if (vec_magnitude(vec_sub(mouse_coord, particle->position)) <= hover_radius) {
                hover_index = i;
                break;
            }

            if (i == particles.count - 1)
                hover_index = -1;
        }
    }

    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        left_mouse_down = true;
        if (hover_index != -1) {
            selected_index = hover_index;
        } 
    } else if (left_mouse_down && IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
        left_mouse_down = false;
        selected_index = -1;
        /*Vec2 impulse_direction = vec_sub(particles.items[0].position, mouse_coord);*/
        /*float impulse_magnitude = vec_magnitude(impulse_direction) * 5;*/
        /*particles.items[0].velocity = vec_mult(vec_normalize(impulse_direction), impulse_magnitude);*/
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
    for (int i = 0; i < particles.count; i++) {
        Particle* particle = &particles.items[i];

        // force from arrow keys
        particle_add_force(particle, push_force);

        // weight
        Vec2 weight = VEC2(0.0, particle->mass * 9.8 * PIXELS_PER_METER);
        particle_add_force(particle, weight);

        /*Vec2 friction = force_generate_friction(particle, 5 * PIXELS_PER_METER);*/
        /*particle_add_force(particle, friction);*/

        // drag
        Vec2 drag = force_generate_drag(particle, 0.001);
        particle_add_force(particle, drag);

        bool first_particle = i == 0;
        bool last_particle = i == (particles.count - 1);
        if (first_particle) {
            // apply spring force to the first particle
            Vec2 spring_force = force_generate_spring_anchor(&particles.items[0], anchor, rest_length, k);
            particle_add_force(&particles.items[0], spring_force);
        }

        if (!last_particle) {
            Particle* next_particle = &particles.items[i + 1];
            Vec2 spring_force_this = force_generate_spring_particle(particle, next_particle, rest_length, k);
            particle_add_force(particle, spring_force_this);
            Vec2 spring_force_other = force_generate_spring_particle(next_particle, particle, rest_length, k);
            particle_add_force(next_particle, spring_force_other);
        }
    }

    // integrate forces 
    for (int i = 0; i < particles.count; i++) {
        Particle* particle = &particles.items[i];
        if (i == selected_index) {
            particle->position = mouse_coord;
        } else {
            particle_integrate(particle, delta_time);
        }
    }

    // limit particles inside window boundaries
    /*for (int i = 0; i < particles.count; i++) {*/
    /*    Particle* particle = &particles.items[i];*/
    /**/
    /*    if (particle->position.y >= WINDOW_HEIGHT - particle->radius) {*/
    /*        particle->position.y = WINDOW_HEIGHT - particle->radius;*/
    /*        particle->velocity.y *= -1;*/
    /*    }*/
    /**/
    /*    if (particle->position.y < particle->radius) {*/
    /*        particle->position.y = particle->radius;*/
    /*        particle->velocity.y *= -1;*/
    /*    }*/
    /**/
    /*    if (particle->position.x >= WINDOW_WIDTH - particle->radius) {*/
    /*        particle->position.x = WINDOW_WIDTH - particle->radius;*/
    /*        particle->velocity.x *= -1;*/
    /*    }*/
    /**/
    /*    if (particle->position.x < particle->radius) {*/
    /*        particle->position.x = particle->radius;*/
    /*        particle->velocity.x *= -1;*/
    /*    }*/
    /*}*/
}

void render() {
    begin_frame();
    clear_screen(0x056263FF);

    // particles
    for (int i = 0; i < particles.count; i++) {
        Particle* particle = &particles.items[i];
        bool first_particle = i == 0;
        bool last_particle = i == (particles.count - 1);
        if (first_particle) {
            // first spring
            draw_line(anchor.x, anchor.y, particle->position.x, particle->position.y, 0x313131FF);
            // anchor
            draw_fill_circle(anchor.x, anchor.y, 5, 0x551100FF);
        }
        if (!last_particle) {
            Particle* next_particle = &particles.items[i + 1];
            draw_line(particle->position.x, particle->position.y,
                    next_particle->position.x, next_particle->position.y, 0x313131FF);
        }
        // particle
        draw_fill_circle(particle->position.x, particle->position.y, particle->radius, 0xFFFFFFFF);

        if (hover_index == i)
            draw_circle(particle->position.x, particle->position.y, hover_radius, 0xFF0000FF);
    }

    /*if (left_mouse_down) {*/
    /*    Vec2 particle_pos = particles.items[0].position;*/
    /*    draw_line(mouse_coord.x, mouse_coord.y, particle_pos.x, particle_pos.y, 0xFF0000FF);*/
    /*}*/

    end_frame();
}

