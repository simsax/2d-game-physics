#ifndef ARRAY_H
#define ARRAY_H

#include <stdlib.h>
#include <stdint.h>

#include "vec2.h"

typedef struct {
    uint32_t capacity;
    uint32_t count;
    int* items;
} IntArray;

typedef struct {
    uint32_t capacity;
    uint32_t count;
    Vec2* items;
} Vec2Array;

typedef struct Body Body;

typedef struct {
    uint32_t capacity;
    uint32_t count;
    Body* items;
} BodyArray;

#define START_CAPACITY 8

#define DA_APPEND(xs, x)                                                                    \
    do {                                                                                    \
        if ((xs)->count >= (xs)->capacity) {                                                \
            if ((xs)->capacity == 0)                                                        \
                (xs)->capacity = START_CAPACITY;                                            \
            else                                                                            \
                (xs)->capacity *= 2;                                                        \
            (xs)->items = realloc((xs)->items, (xs)->capacity * sizeof(*(xs)->items));      \
        }                                                                                   \
        (xs)->items[(xs)->count++] = (x);                                                   \
    } while (0)                                                                             

#define DA_NULL { .capacity = 0, .count = 0, .items = NULL }
#define DA_FREE(xs) free((xs)->items)

#endif // ARRAY_H
