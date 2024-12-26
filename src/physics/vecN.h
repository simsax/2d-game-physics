#ifndef VECN_H
#define VECN_H

typedef struct {
    int N;
    float* data;
} VecN;

VecN vecN_create(int length);
void vecN_free(VecN vec);
void vecN_zero(VecN vec);
void vecN_sub(VecN this_, VecN other);
void vecN_add(VecN this_, VecN other);
void vecN_mult(VecN this_, float f);
float vecN_dot(VecN a, VecN b);

#endif // VECN_H
