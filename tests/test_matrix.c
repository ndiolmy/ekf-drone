#include "matrix.h"
#include <stdio.h>


int main(void) {
	printf("TEST matrix multiplication :\n");
	Matrix I,A,C;
	int n =2;
	mat_eye(&I,n);
	mat_print(&I, "I");
	A.rows=A.cols=C.cols=C.rows=2;
	for(int i=0;i<n;i++){
		for(int j=0;j<n;j++) {
			A.data[i][j] = 2;
		}
	}	
	mat_print(&A, "A");
	mat_mul(&C,&A,&I);
	mat_print(&C, "C");
	if (mat_equal(&A,&C,1e-5f)==1) printf("ok\n");
	else printf("FAUX\n");
	
	printf("TEST matrix inverse :\n");
	
	Matrix Iinv, Itest;
	mat_zero(&Iinv,n);
	mat_zero(&Itest,n);
	
	mat_inv(&Iinv,&I);
	mat_print(&I,"I");
	mat_print(&Iinv,"Iinv");
	
	mat_mul(&Itest,&I,&Iinv);
	
	if (mat_equal(&I,&Itest,1e-5f)==1) printf("ok\n");
	else printf("FAUX\n");
	
	Matrix D, Dinv, Dtest;
	mat_zero(&D,n);
	mat_zero(&Dinv,n);
	mat_zero(&Dtest,n);
	
	D.data[0][1] = D.data[1][0] = 2;
	D.data[0][0] = 4;
	D.data[1][1] = 3;
	
	mat_inv(&Dinv,&D);
	mat_print(&D,"D");
	mat_print(&Dinv,"Dinv");
	
	mat_mul(&Dtest,&D,&Dinv);
	
	if (mat_equal(&Dtest,&I,1e-5f)==1) printf("ok\n");
	else printf("FAUX\n");
	
	
	
	
}	
	
	

