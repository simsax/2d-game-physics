#ifndef PARTICLE_H
#define PARTICLE_H

#include "vec2.h"

typedef struct {
    Vec2 position;
    Vec2 velocity;
    Vec2 acceleration;
    float mass;
} Particle;

void init_particle(Particle* particle, int x, int y, float mass);

#endif //  PARTICLE_H
