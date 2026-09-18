#include "matrix.h"
#include <math.h>
#include <stdio.h>

void mat_add(Matrix* C, const Matrix* A, const Matrix* B) {
	C->rows = A->rows;
	C->cols = A->cols;
	for(int i=0;i<C->rows;i++){
		for(int j=0;j<C->cols;j++) {
			C->data[i][j] = A->data[i][j] + B->data[i][j];
		}
	}
	
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
	At->rows = A->cols;
	At->cols = A->rows;		
	for(int i=0;i<At->rows;i++){
		for(int j=0;j<At->cols;j++) {
			At->data[i][j] = A->data[j][i];
		}
	}		
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

void mat_zero(Matrix *A, int n,int m) {
	A->cols=m, A->rows=n;
	for (int i=0;i<n;i++) {
		for (int j=0;j<m;j++) {
			A->data[i][j] = 0;
		}
	}
}			
	
	
	
void mat_inv(Matrix *Ainv, const Matrix *A) {
	//calculate L as L*Ltrans = A
	int n = A->rows;
	
	Matrix L;
	mat_zero(&L,n,n	);
	
	//calculate column per column
	for (int j = 0; j < n; j++) {
    
		float sum = 0;
		for (int k = 0; k < j; k++)
			sum += L.data[j][k] * L.data[j][k];
		L.data[j][j] = sqrtf(A->data[j][j] - sum);

    
		for (int i = j + 1; i < n; i++) {
			sum = 0;
			for (int k = 0; k < j; k++)
				sum += L.data[i][k] * L.data[j][k];
			L.data[i][j] = (A->data[i][j] - sum) / L.data[j][j];
		}
	}
		
		
	
	//calculate Ainv by calculating L*Y=I with Y = Ltrans*Ainv then 
	//Ltrans*Ainv = Y
	Matrix I;
	mat_eye(&I,n);
	Matrix Y;
	mat_zero(&Y,n,n);
	mat_zero(Ainv,n,n);
	
	//calculus of Y
	for (int j=0;j<n;j++) {
		for (int i=0;i<n;i++) {
			float sum = 0;
			for (int k=0;k<i;k++) {
				sum += L.data[i][k]*Y.data[k][j];
			}
			if (i!=j) {
				Y.data[i][j] = -sum/L.data[i][i];
			}
			else {
				Y.data[i][j] = (1-sum)/L.data[i][i];
			}
		}	
	}
	
	Matrix Ltrans;
	mat_transpose(&Ltrans,&L);
	
	//calculus of Ainv
	
	for (int i=0;i<n;i++) {
		Ainv->data[n-1][i]=Y.data[n-1][i]/Ltrans.data[n-1][n-1];
		for (int k=n-2;k>=0;k--) {
			float sum = 0;
			for (int j=k+1;j<n;j++) {
				sum += Ainv->data[j][i]*Ltrans.data[k][j];
			}
			Ainv->data[k][i] = (Y.data[k][i]-sum)/Ltrans.data[k][k];
		}	
	}
}
	
	
void mat_copy(Matrix *dst, const Matrix* src) {
	int n= src->rows;
	int m= src->cols;
	dst->rows = n;
	dst->cols = m;
	for (int i=0; i<n; i++) {
		for (int j=0; j<m; j++) {
			dst->data[i][j] = src->data[i][j];
		}
	}
}
		
			
				
			
			
	
	
	
	
	
	
	
	
