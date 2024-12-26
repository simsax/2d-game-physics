#include "vecN.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

VecN vecN_create(int length) {
    float* data = malloc(sizeof(float) * length);
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
    memset(vec.data, 0, vec.N);
}

void vecN_sub(VecN this_, VecN other) {
    for (int i = 0; i < this_.N; i++)
        this_.data[i] -= other.data[i];
}

void vecN_add(VecN this_, VecN other) {
    for (int i = 0; i < this_.N; i++)
        this_.data[i] += other.data[i];
}

void vecN_mult(VecN this_, float f) {
    for (int i = 0; i < this_.N; i++)
        this_.data[i] *= f;
}

float vecN_dot(VecN a, VecN b) {
    float sum = 0.0f;
    for (int i = 0; i < a.N; i++)
        sum += a.data[i] * b.data[i];
    return sum;
}
