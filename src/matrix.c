#include "matrix.h"
#include <math.h>
#include <stdio.h>

void mat_add(Matrix* C, const Matrix* A, const Matrix* B) {
	for(int i=0;i<C->rows;i++){
		for(int j=0;j<C->cols;j++) {
			C->data[i][j] = A->data[i][j] + B->data[i][j];
		}
	}
	C->rows = A->rows;
	C->cols = A->cols;
}		
	
void mat_sub(Matrix* C, const Matrix* A, const Matrix* B) {
	for(int i=0;i<A->rows;i++){
		for(int j=0;j<A->cols;j++) {
			C->data[i][j] = A->data[i][j] - B->data[i][j];
		}
	}
	C->rows = A->rows;
	C->cols = A->cols;
}	


void mat_mul(Matrix* C, const Matrix* A, const Matrix* B) {
	for(int i=0;i<A->rows;i++){
		for(int j=0;j<B->cols;j++) {
			float c = 0;
			for(int ia=0;ia<A->cols;ia++){
				c += A->data[i][ia]*B->data[ia][j];
			C->data[i][j] = c;
			}
		}
	}
	C->rows = A->rows;
	C->cols = B->cols;							
}	
	

void mat_transpose(Matrix* At, const Matrix* A) {
	for(int i=0;i<A->rows;i++){
		for(int j=0;j<A->cols;j++) {
			At->data[i][j] = A->data[j][i];
		}
	}		
	At->rows = A->rows;
	At->cols = A->cols;		
}	
	
void mat_eye(Matrix* I, int n) {
	I->rows = n;
	I->cols = n;	
	for(int i=0;i<n;i++){
		for(int j=0;j<n;j++) {
			if(i==j) I->data[i][j] = 1;
			else I->data[i][j] = 0;
		}
	}
}	

int mat_equal(const Matrix *A, const Matrix *B, float tol) {
    if (A->rows != B->rows || A->cols != B->cols) return 0;
    for (int i = 0; i < A->rows; i++)
        for (int j = 0; j < A->cols; j++)
            if (fabsf(A->data[i][j] - B->data[i][j]) > tol)
                return 0;
    return 1;
}
void mat_print(const Matrix *A, const char *name) {
    printf("%s (%dx%d):\n", name, A->rows, A->cols);
    for (int i = 0; i < A->rows; i++) {
        for (int j = 0; j < A->cols; j++)
            printf("%8.4f ", A->data[i][j]);
        printf("\n");
    }
    printf("\n");
}
