#ifndef FORCE_H
#define FORCE_H

#include "particle.h"

Vec2 force_generate_drag(Particle* particle, float k);
Vec2 force_generate_friction(Particle* particle, float k);
Vec2 force_generate_gravitational(Particle* a, Particle* b, float G, float min_dist, float max_dist);
Vec2 force_generate_spring_anchor(Particle* particle, Vec2 anchor, float rest_length, float k);
Vec2 force_generate_spring_particle(Particle* a, Particle* b, float rest_length, float k);

#endif // FORCE_H
