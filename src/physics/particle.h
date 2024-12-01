#ifndef PARTICLE_H
#define PARTICLE_H

#include "vec2.h"

typedef struct {
    Vec2 position;
    Vec2 velocity;
    Vec2 acceleration;
    float mass;
    int radius;
} Particle;

void init_particle(Particle* particle, int x, int y, float mass, int radius);
void integrate_particle(Particle* particle, float dt);

#endif //  PARTICLE_H
