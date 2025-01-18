#ifndef VECN_H
#define VECN_H

typedef struct {
    int N;
    float* data;
} VecN;

typedef struct Arena Arena;

VecN vecN_create(int length, Arena* arena);
void vecN_free(VecN vec);
void vecN_zero(VecN vec);
VecN vecN_sub(VecN a, VecN b, Arena* arena);
VecN vecN_add(VecN a, VecN b, Arena* arena);
VecN vecN_mult(VecN a, float f, Arena* arena);
float vecN_dot(VecN a, VecN b);
void vecN_print(VecN vec);

#endif // VECN_H
