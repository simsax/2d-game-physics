#ifndef VECN_H
#define VECN_H

typedef struct {
    int N;
    float* data;
} VecN;

VecN vecN_create(int length);
void vecN_free(VecN vec);
void vecN_zero(VecN vec);
VecN vecN_sub(VecN a, VecN b);
VecN vecN_add(VecN a, VecN b);
VecN vecN_mult(VecN a, float f);
float vecN_dot(VecN a, VecN b);
void vecN_print(VecN vec);

#endif // VECN_H
