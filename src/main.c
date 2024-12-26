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


    // matrix tests
    printf("\n---------------------\n");
    MatMN m = matMN_create(3, 4);
    mat_print(m);
    matMN_zero(m);
    mat_print(m);

    for (int i = 0; i < m.M; i++) {
        for (int j = 0; j < m.N; j++) {
            MAT_SET(m, i, j, i + j);
        }
    }
    mat_print(m);
    MatMN t = matMN_transpose(m);
    mat_print(t);

    VecN v3 = vecN_create(4);
    for (int i = 0; i < v3.N; i++)   
        v3.data[i] = i * 2 + 1;
    vec_print(v3);
    VecN res = matMN_mult_vec(m, v3);
    vec_print(res);

    MatMN resm = matMN_mult_mat(m, t);
    mat_print(resm);

    vecN_free(v);
    vecN_free(v2);
    vecN_free(v3);
    vecN_free(res);
    matMN_free(m);
    matMN_free(t);
    matMN_free(resm);
}

// TODO: test on non-symmetric random matrices
// https://www.calculator.net/matrix-calculator.html

int main(void)
{
    matrix_tests();
    return 0;
}

