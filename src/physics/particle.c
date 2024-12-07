#include "particle.h"

Particle particle_create(int x, int y, float mass, int radius) {
    return (Particle) {
        .position = VEC2(x, y),
        .velocity = VEC2(0, 0), 
        .acceleration = VEC2(0, 0),
        .sum_forces = VEC2(0, 0),
        .inv_mass = mass != 0 ? 1.0 / mass : 0,
        .radius = radius
    };
}

void particle_init(Particle* particle, int x, int y, float mass, int radius) {
    particle->position = (Vec2) { .x = x, .y = y };
    particle->inv_mass = mass != 0 ? 1.0 / mass : 0;
    particle->velocity = VEC2(0, 0);
    particle->acceleration = VEC2(0, 0);
    particle->radius = radius;
}

void particle_integrate(Particle* particle, float dt) {
    // find acceleration based on the forces being applied and the mass
    particle->acceleration = vec_mult(particle->sum_forces, particle->inv_mass);

    // integrate acceleration to find new velocity
    particle->velocity = vec_add(particle->velocity, vec_scale(particle->acceleration, dt));

    // integrate velocity to find new position
    particle->position = vec_add(particle->position, vec_scale(particle->velocity, dt));

    particle_clear_forces(particle);
}

void particle_add_force(Particle* particle, Vec2 force) {
    particle->sum_forces = vec_add(particle->sum_forces, force);
}

void particle_clear_forces(Particle* particle) {
    particle->sum_forces = VEC2(0, 0);
}
