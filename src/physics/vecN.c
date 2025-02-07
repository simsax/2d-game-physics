#include "vecN.h"
#include "memory.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>


VecN vecN_create(int length, Arena* arena) {
    float* data = NULL;
    if (arena != NULL) {
        data = arena_alloc(arena, length * sizeof(float));
    } else {
        data = malloc(length * sizeof(float));
    }
    if (data == NULL) {
        printf("ERROR: Mem allocation failed, quitting.\n");
        exit(1);
    }
    return (VecN) {
        .N = length,
        .data = data
    };
}

void vecN_free(VecN vec) {
    free(vec.data);
}

void vecN_zero(VecN vec) {
    memset(vec.data, 0, vec.N * sizeof(float));
}

VecN vecN_sub(VecN a, VecN b, Arena* arena) {
    VecN res = vecN_create(a.N, arena);
    for (int i = 0; i < a.N; i++)
        res.data[i] = a.data[i] - b.data[i];
    return res;
}

VecN vecN_add(VecN a, VecN b, Arena* arena) {
    VecN res = vecN_create(a.N, arena);
    for (int i = 0; i < a.N; i++)
        res.data[i] = a.data[i] + b.data[i];
    return res;
}

VecN vecN_mult(VecN a, float f, Arena* arena) {
    VecN res = vecN_create(a.N, arena);
    for (int i = 0; i < a.N; i++)
        res.data[i] = a.data[i] * f;
    return res;
}

float vecN_dot(VecN a, VecN b) {
    float sum = 0.0f;
    for (int i = 0; i < a.N; i++)
        sum += a.data[i] * b.data[i];
    return sum;
}

void vecN_print(VecN vec) {
    printf("[\n");
    for (int i = 0; i < vec.N; i++) {
        printf("    %f\n", (double)vec.data[i]);
    }
    printf("]\n");
}
