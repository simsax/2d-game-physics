#include "particle.h"

void init_particle(Particle* particle, int x, int y, float mass) {
    particle->position = (Vec2) { .x = x, .y = y };
    particle->mass = mass;
    particle->velocity = vec2(0);
    particle->acceleration = vec2(0);
}
