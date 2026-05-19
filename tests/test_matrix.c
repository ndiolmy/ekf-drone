#include "matrix.h"
#include <stdio.h>


int main(void) {
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
}
