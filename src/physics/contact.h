#ifndef CONTACT_H
#define CONTACT_H

#include "vec2.h"
#include "body.h"

typedef struct {
    Body* a;
    Body* b;

    Vec2 start;
    Vec2 end;

    Vec2 normal;
    float depth;
} Contact;

void contact_resolve_penetration(Contact* contact);

#endif // CONTACT_H
