#ifndef MATMN_H
#define MATMN_H

#include "vecN.h"

typedef struct {
    int M; // rows
    int N; // cols
    float *data;
} MatMN;

#define MAT_GET(mat, i, j) ((mat).data[(i) * (mat).N + (j)])
#define MAT_SET(mat, i, j, val) ((mat).data[(i) * (mat).N + (j)] = (val))

MatMN matMN_create(int num_rows, int num_cols);
void matMN_free(MatMN mat);
void matMN_zero(MatMN mat);
MatMN matMN_transpose(MatMN mat);
VecN matMN_mult_vec(MatMN mat, VecN vec);
MatMN matMN_mult_mat(MatMN a, MatMN b);

#endif // MATMN_H
