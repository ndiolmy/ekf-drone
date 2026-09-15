#ifndef MATRIX_H
#define MATRIX_H

#define MAX_N 10

typedef struct {
    float data[MAX_N][MAX_N];
    int rows, cols;
} Matrix;

void mat_add(Matrix *C, const Matrix *A, const Matrix *B);
void mat_sub(Matrix *C, const Matrix *A, const Matrix *B);
void mat_mul(Matrix *C, const Matrix *A, const Matrix *B);
void mat_transpose(Matrix *At, const Matrix *A);
void mat_inv(Matrix *Ainv, const Matrix *A);
void mat_eye(Matrix *I, int n);
void mat_copy(Matrix *dst, const Matrix *src);
int mat_equal(const Matrix *A, const Matrix *B, float tol);
void mat_print(const Matrix *A, const char *name);
void mat_zero(Matrix* A,int n, int m);

#endif
