#include "particle.h"

void init_particle(Particle* particle, int x, int y, float mass, int radius) {
    particle->position = (Vec2) { .x = x, .y = y };
    particle->mass = mass;
    particle->velocity = vec2(0);
    particle->acceleration = vec2(0);
    particle->radius = radius;
}

void integrate_particle(Particle* particle, float dt) {
    particle->velocity = vec_add(particle->velocity, vec_scale(particle->acceleration, dt));
    particle->position = vec_add(particle->position, vec_scale(particle->velocity, dt));
}
