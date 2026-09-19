#include "imu_model.h"
#include <math.h>

void imu_f(Matrix *x_out, const Matrix *x, const Matrix *u) {
    float px  = x->data[0][0];
    float py  = x->data[1][0];
    float pz  = x->data[2][0];
    float vx  = x->data[3][0];
    float vy  = x->data[4][0];
    float vz  = x->data[5][0];
    float phi = x->data[6][0];
    float tht = x->data[7][0];
    float psi = x->data[8][0];

    float ax = u->data[0][0];
    float ay = u->data[1][0];
    float az = u->data[2][0];
    float p  = u->data[3][0];
    float q  = u->data[4][0];
    float r  = u->data[5][0];

    float sp = sinf(phi), cp = cosf(phi);
    float ct = cosf(tht), tt = tanf(tht);
    float dt = IMU_DT;

    // position
    x_out->data[0][0] = px + vx * dt;
    x_out->data[1][0] = py + vy * dt;
    x_out->data[2][0] = pz + vz * dt;

    // velocities
    x_out->data[3][0] = vx + ax * dt;
    x_out->data[4][0] = vy + ay * dt;
    x_out->data[5][0] = vz + (az - GRAVITY) * dt;

    // Euler angles
    x_out->data[6][0] = phi + (p + q * sp * tt + r * cp * tt) * dt;
    x_out->data[7][0] = tht + (q * cp - r * sp) * dt;
    x_out->data[8][0] = psi + (q * sp + r * cp) / ct * dt;

    x_out->rows = 9;
    x_out->cols = 1;
}

void imu_F_jac_numerical(Matrix *F, const Matrix *x, const Matrix *u, float eps) {
    Matrix f_base, f_plus, x_plus;
    imu_f(&f_base, x, u);

    mat_zero(F, 9, 9);
    for (int j = 0; j < 9; j++) {
        mat_copy(&x_plus, x);
        x_plus.data[j][0] += eps;
        imu_f(&f_plus, &x_plus, u);
        for (int i = 0; i < 9; i++)
            F->data[i][j] = (f_plus.data[i][0] - f_base.data[i][0]) / eps;
    }
    F->rows = 9;
    F->cols = 9;
}
