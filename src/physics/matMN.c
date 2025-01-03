#include "matMN.h"
#include "vecN.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// TODO: get rid of heap allocations once I figure out the usage, compare with cglm

MatMN matMN_create(int num_rows, int num_cols) {
    float *data = malloc(sizeof(float) * num_rows * num_cols);
    if (data == NULL) {
        printf("ERROR: Mem allocation failed, quitting.\n");
        exit(1);
    }
    return (MatMN) {
        .M = num_rows,
        .N = num_cols,
        .data = data
    };
}

void matMN_free(MatMN mat) {
    free(mat.data);
}

void matMN_zero(MatMN mat) {
    memset(mat.data, 0, mat.M * mat.N * sizeof(float));
}

MatMN matMN_transpose(MatMN mat) {
    MatMN trans = matMN_create(mat.N, mat.M);
    for (int i = 0; i < mat.M; i++) {
        for (int j = 0; j < mat.N; j++) {
            float val = MAT_GET(mat, i, j);
            MAT_SET(trans, j, i, val);
        }
    }
    return trans;
}

VecN matMN_mult_vec(MatMN mat, VecN vec) {
    if (mat.N != vec.N) {
        printf("ERROR: cannot multiply matrix of dimensions (%d, %d) with vector of dimensions (%d, 1).\n",
                mat.M, mat.N, vec.N);
        exit(1);
    }
    VecN result = vecN_create(mat.M);
    for (int i = 0; i < mat.M; i++) {
        float val = 0;
        // dot prod of row i of mat with vec
        for (int j = 0; j < mat.N; j++) {
            val += MAT_GET(mat, i, j) * vec.data[j];
        }
        result.data[i] = val;
    }
    return result;
}

MatMN matMN_mult_mat(MatMN a, MatMN b) {
    if (a.N != b.M) {
        printf("ERROR: cannot multiply matrix of dimensions (%d, %d) with matrix of dimensions (%d, %d).\n",
                a.M, a.N, b.M, b.N);
        exit(1);
    }
    MatMN result = matMN_create(a.M, b.N);
    for (int i = 0; i < result.M; i++) {
        for (int j = 0; j < result.N; j++) {
            float val = 0;
            // dot prod of row i of a with col j of b
            for (int k = 0; k < a.N; k++) {
                val += MAT_GET(a, i, k) * MAT_GET(b, k, j);
            }
            MAT_SET(result, i, j, val);
        }
    }
    return result;
}

VecN matMN_solve_gauss_seidel(MatMN a, VecN b) {
    int N = b.N;
    VecN X = vecN_create(N);
    vecN_zero(X);

    for (int iterations = 0; iterations < N; iterations++) {
        for (int i = 0; i < N; i++) {
            float a_ii = MAT_GET(a, i, i);
            float ai_dot_x = 0.0f;
            for (int j = 0; j < a.N; j++) {
                ai_dot_x += MAT_GET(a, i, j) * X.data[j];
            }
            if (a_ii != 0.0f) {
                X.data[i] += (b.data[i] / a_ii) - (ai_dot_x / a_ii);
            }
        }
    }

    return X;
}

void matMN_print(MatMN m) {
    printf("[\n");
    for (int i = 0; i < m.M; i++) {
        printf("    ");
        for (int j = 0; j < m.N; j++) {
            float val = MAT_GET(m, i, j);
            printf("%.2f ", val);
        }
        printf("\n");
    }
    printf("]\n");
}
