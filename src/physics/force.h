#ifndef FORCE_H
#define FORCE_H

#include "particle.h"

Vec2 force_generate_drag(Particle* particle, float k);
Vec2 force_generate_friction(Particle* particle, float k);
Vec2 force_generate_gravitational(Particle* a, Particle* b, float G, float min_dist, float max_dist);

#endif // FORCE_H
