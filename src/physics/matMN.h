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

MatMN matMN_create(int num_rows, int num_cols, Arena* arena);
void matMN_free(MatMN mat);
void matMN_zero(MatMN mat);
MatMN matMN_transpose(MatMN mat, Arena* arena);
VecN matMN_mult_vec(MatMN mat, VecN vec, Arena* arena);
MatMN matMN_mult_mat(MatMN a, MatMN b, Arena* arena);
VecN matMN_solve_gauss_seidel(MatMN a, VecN b, Arena* arena);
void matMN_print(MatMN m);

#endif // MATMN_H
