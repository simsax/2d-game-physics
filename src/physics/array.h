#ifndef ARRAY_H
#define ARRAY_H

#include <stdlib.h>
#include <stdint.h>

typedef struct {
    uint32_t capacity;
    uint32_t count;
    int* items;
} IntArray;

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
        }                                                                                   \
        (xs)->items[(xs)->count++] = (x);                                                   \
    } while (0)                                                                             

#define DA_NEXT_PTR(xs)                                                                 \
    (((xs)->count >= (xs)->capacity) ?                                                  \
     ((xs)->capacity = ((xs)->capacity == 0 ? START_CAPACITY : (xs)->capacity * 2),     \
      (xs)->items = realloc((xs)->items, (xs)->capacity * sizeof(*(xs)->items))) :      \
      NULL,                                                                             \
      &((xs)->items[(xs)->count++]))

#define DA_NULL { .capacity = 0, .count = 0, .items = NULL }
#define DA_FREE(xs) free((xs)->items)

#endif // ARRAY_H
