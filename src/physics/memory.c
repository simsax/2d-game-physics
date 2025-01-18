#include "memory.h"
#include <stdio.h>

void arena_init(Arena* arena, size_t size) {
    arena->data = malloc(size);
    if (arena->data == NULL) {
        printf("ERROR: out of memory, aborting.\n");
        exit(1);
    }
    arena->count = 0;
    arena->capacity = size;
}

void* arena_alloc(Arena* arena, size_t size) {
    if (arena->count + size >= arena->capacity) {
        // TODO: I can't realloc, I'll lose all my pointers
        printf("ERROR: out of memory, aborting.\n");
        exit(1);

        arena->capacity = (arena->count + size) * 2;
        arena->data = realloc(arena->data, arena->capacity);
        if (arena->data == NULL) {
            printf("ERROR: out of memory, aborting.\n");
            exit(1);
        }
    }
    void* start_of_block = &((char *)arena->data)[arena->count];
    arena->count += size;
    return start_of_block;
}

void arena_free(Arena* arena, void* ptr) {
    if (ptr == NULL) {
        free(arena->data);
    } else {
        // reset count to difference between ptr and base pointer
        // keep in mind that there are garbage values to be cleaned each frame and others that should be kept alive
    }
}
