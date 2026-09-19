#include "matrix.h"
#include "ekf.h"

void ekf_init(EKF *ekf, const Matrix *x0, const Matrix *P0) {
	mat_copy(&ekf->x, x0);
    mat_copy(&ekf->P, P0);
}

void ekf_predict(EKF *ekf, const Matrix *u, float eps) {
    //prediction
    Matrix x_new, F;
    ekf->f(&x_new, &ekf->x,u);
    
    
    ekf->F_jac(&F,&ekf->x,u,eps);
    
    ekf->x = x_new;
    
    Matrix F_trans, X, Y;
    mat_transpose(&F_trans, &F);
    mat_mul(&Y, &ekf->P, &F_trans);
    mat_mul(&X, &F, &Y);
    mat_add(&ekf->P, &X, &ekf->Q);
    
    
}

void ekf_update(EKF *ekf, const Matrix *z) {
    //innovation y 
    Matrix y, z_pred, H;
    ekf->h(&z_pred, &ekf->x);
    mat_sub(&y,z, &z_pred);
    
    ekf->H_jac(&H, &ekf->x);
    
    //matrice d'innovation S
    Matrix H_trans, X, Y, S;
    mat_transpose(&H_trans, &H);
    mat_mul(&Y, &ekf->P, &H_trans);
    mat_mul(&X, &H, &Y);
    mat_add(&S, &X, &ekf->R);
    
    //gain Kalman K
    Matrix K, A, Sinv;
    mat_inv(&Sinv, &S);
    mat_mul(&A, &H_trans, &Sinv);
    mat_mul(&K, &ekf->P, &A);
    
    //mise à jour de l'état
    Matrix x_new, B;
    mat_mul(&B, &K, &y);
    mat_add(&x_new, &ekf->x, &B);
    ekf->x = x_new;
    
    //mise à jour de P
    Matrix C, D, I;
    mat_eye(&I, K.rows);
    mat_mul(&C, &K, &H);
    mat_sub(&D,&I , &C);
    mat_mul(&ekf->P, &D, &ekf->P);   
    
}
    
    
    
    
    
    
    
    
    
    

