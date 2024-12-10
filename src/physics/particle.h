#ifndef PARTICLE_H
#define PARTICLE_H

#include "vec2.h"

typedef struct {
    Vec2 position;
    Vec2 old_position;
    Vec2 velocity;
    Vec2 acceleration;
    Vec2 sum_forces;
    float inv_mass;
    int radius;
} Particle;

Particle particle_create(int x, int y, float mass, int radius);
void particle_init(Particle* particle, int x, int y, float mass, int radius);
void particle_integrate(Particle* particle, float dt);
void particle_integrate_verlet(Particle* particle, float dt);
void particle_add_force(Particle* particle, Vec2 force);
void particle_clear_forces(Particle* particle);

#endif //  PARTICLE_H
