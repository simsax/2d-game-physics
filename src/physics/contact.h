#ifndef CONTACT_H
#define CONTACT_H

#include "vec2.h"

typedef struct {
    int a_index;
    int b_index;

    Vec2 start;
    Vec2 end;

    Vec2 normal;
    float depth;
} Contact;

#endif // CONTACT_H
