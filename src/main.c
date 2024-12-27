#include "application.h"

void run() {
    setup();

    while (running) {
        input();
        update();
        render();
    }

    destroy();
}

#include <stdio.h>
#include "physics/vecN.h"
#include "physics/matMN.h"

void vec_print(VecN v) {
    printf("[ ");
    for (int i = 0; i < v.N; i++)   
        printf("%f ", v.data[i]);
    printf("]\n");
}

void mat_print(MatMN m) {
    printf("[\n");
    for (int i = 0; i < m.M; i++) {
        printf("    ");
        for (int j = 0; j < m.N; j++) {
            printf("%f ", MAT_GET(m, i, j));
        }
        printf("\n");
    }
    printf("]\n");
}

void matrix_tests() {
    // vector tests
    VecN v = vecN_create(6);
    vec_print(v);
    vecN_zero(v);
    vec_print(v);
    for (int i = 0; i < v.N; i++)   
        v.data[i] = i + 1;
    vec_print(v);
    VecN v2 = vecN_create(6);
    for (int i = 0; i < v2.N; i++)   
        v2.data[i] = 2;
    vecN_sub(v, v2);
    vec_print(v);
    vecN_add(v, v2);
    vec_print(v);
    vecN_mult(v, 3);
    vec_print(v);
    float d = vecN_dot(v, v2);
    printf("Dot is: %f\n", d);

    float values_a[] = {
        4, 5, 7, 3, 0, 1, 8,
        6, 5, 3, 7, 1, 7, 8,
        4, 0, 6, 8, 6, 0, 4,
        4, 6, 5, 0, 0, 5, 9,
        0, 9, 8, 0, 1, 9, 3
    };

    float values_b[] = {
        2, 8, 4, 0,
        9, 3, 2, 0,
        8, 9, 1, 3,
        0, 6, 0, 7,
        4, 7, 4, 3,
        3, 1, 8, 1,
        8, 8, 5, 3
    };

    // matrix tests
    printf("\n---------------------\n");
    MatMN a = matMN_create(5, 7);
    int k = 0;
    for (int i = 0; i < a.M; i++) {
        for (int j = 0; j < a.N; j++) {
            MAT_SET(a, i, j, values_a[k++]);
        }
    }
    mat_print(a);

    MatMN b = matMN_create(7, 4);
    k = 0;
    for (int i = 0; i < b.M; i++) {
        for (int j = 0; j < b.N; j++) {
            MAT_SET(b, i, j, values_b[k++]);
        }
    }
    mat_print(b);

    MatMN resm = matMN_mult_mat(a, b);
    mat_print(resm);

    vecN_free(v);
    vecN_free(v2);
    matMN_free(a);
    matMN_free(b);
    matMN_free(resm);
}

// TODO: test on non-symmetric random matrices
// https://www.calculator.net/matrix-calculator.html

int main(void)
{
    matrix_tests();
    return 0;
}

