#include "particle.h"

Particle particle_create(int x, int y, float mass, int radius) {
    return (Particle) {
        .position = VEC2(x, y),
        .old_position = VEC2(x, y),
        .velocity = VEC2(0, 0), 
        .acceleration = VEC2(0, 0),
        .sum_forces = VEC2(0, 0),
        .inv_mass = mass != 0 ? 1.0 / mass : 0,
        .radius = radius
    };
}

void particle_init(Particle* particle, int x, int y, float mass, int radius) {
    particle->position = (Vec2) { .x = x, .y = y };
    particle->old_position = particle->position;
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

void particle_integrate_verlet(Particle* particle, float dt) {
    // find velocity based on current and old position
    Vec2 velocity = vec_sub(particle->position, particle->old_position);
    particle->old_position = particle->position;

    // divide force by the mass to find acceleration
    particle->acceleration = vec_mult(particle->sum_forces, particle->inv_mass);

    // verlet to find new position
    // x_n = x_n + (x_n - x_old_n) + a * dt^2
    Vec2 position_update = vec_add(velocity, vec_mult(particle->acceleration, dt * dt));
    particle->position = vec_add(particle->position, position_update);
}

void particle_add_force(Particle* particle, Vec2 force) {
    particle->sum_forces = vec_add(particle->sum_forces, force);
}

void particle_clear_forces(Particle* particle) {
    particle->sum_forces = VEC2(0, 0);
}
