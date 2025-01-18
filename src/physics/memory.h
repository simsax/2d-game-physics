#ifndef MEMORY_H
#define MEMORY_H

#include <stdint.h>
#include <stdlib.h>

typedef struct Arena {
    void* data;
    size_t capacity;
    size_t count;
} Arena;

void arena_init(Arena* arena, size_t size);
void* arena_alloc(Arena* arena, size_t size);
void arena_free(Arena* arena, void* ptr);

#endif // MEMORY_H
