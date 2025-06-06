#ifndef ARRAY_H
#define ARRAY_H

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

typedef struct {
    uint32_t capacity;
    uint32_t count;
    int* items;
} IntArray;

typedef struct {
    uint32_t capacity;
    uint32_t count;
    float* items;
} FloatArray;

#define START_CAPACITY 8

// idea stolen from Tsoding (https://gist.github.com/rexim/b5b0c38f53157037923e7cdd77ce685d)
#define DA_APPEND(xs, x)                                                                    \
    do {                                                                                    \
        if ((xs)->count >= (xs)->capacity) {                                                \
            if ((xs)->capacity == 0)                                                        \
                (xs)->capacity = START_CAPACITY;                                            \
            else                                                                            \
                (xs)->capacity *= 2;                                                        \
            (xs)->items = realloc((xs)->items, (xs)->capacity * sizeof(*(xs)->items));      \
            if ((xs)->items == NULL) {                                                      \
                printf("ERROR: out of memory, aborting.\n");                                \
                exit(1);                                                                    \
            }                                                                               \
        }                                                                                   \
        (xs)->items[(xs)->count++] = (x);                                                   \
    } while (0)                                                                             

#define DA_NEXT_PTR(xs)                                                                 \
    (((xs)->count >= (xs)->capacity) ?                                                  \
     ((xs)->capacity = ((xs)->capacity == 0 ? START_CAPACITY : (xs)->capacity * 2),     \
      (xs)->items = realloc((xs)->items, (xs)->capacity * sizeof(*(xs)->items)),        \
      ((xs)->items == NULL ? exit(69) : (void) 0)                                       \
      ) :                                                                               \
      (void) 0,                                                                         \
      &((xs)->items[(xs)->count++]))

#define DA_NULL { .capacity = 0, .count = 0, .items = NULL }

// pointer must be set to NULL otherwise next realloc on this pointer will be undefined
#define DA_FREE(xs)                                                                     \
    do {                                                                                \
        free((xs)->items);                                                              \
        (xs)->capacity = 0;                                                             \
        (xs)->count = 0;                                                                \
        (xs)->items = NULL;                                                             \
    } while (0)                                                                         \


#endif // ARRAY_H
