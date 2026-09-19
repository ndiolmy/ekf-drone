#include "sensor_models.h"
#include <math.h>

float angle_wrap(float angle) {
    while (angle >  (float)M_PI) angle -= 2.0f * (float)M_PI;
    while (angle < -(float)M_PI) angle += 2.0f * (float)M_PI;
    return angle;
}

/* GPS */
void gps_h(Matrix *z_pred, const Matrix *x) {
    z_pred->data[0][0] = x->data[0][0];
    z_pred->data[1][0] = x->data[1][0];
    z_pred->data[2][0] = x->data[2][0];
    z_pred->rows = 3; z_pred->cols = 1;
}
void gps_H_jac(Matrix *H, const Matrix *x) {
    (void)x;
    mat_zero(H, 3, 9);
    H->data[0][0] = 1.0f;
    H->data[1][1] = 1.0f;
    H->data[2][2] = 1.0f;
}
void gps_R_init(Matrix *R, float sigma_pos) {
    mat_zero(R, 3, 3);
    float v = sigma_pos * sigma_pos;
    R->data[0][0] = v; R->data[1][1] = v; R->data[2][2] = v;
}

/* Barometer */
void baro_h(Matrix *z_pred, const Matrix *x) {
    z_pred->data[0][0] = x->data[2][0];
    z_pred->rows = 1; z_pred->cols = 1;
}
void baro_H_jac(Matrix *H, const Matrix *x) {
    (void)x;
    mat_zero(H, 1, 9);
    H->data[0][2] = 1.0f;
}
void baro_R_init(Matrix *R, float sigma_alt) {
    mat_zero(R, 1, 1);
    R->data[0][0] = sigma_alt * sigma_alt;
}

/* Magnetometer */
void mag_h(Matrix *z_pred, const Matrix *x) {
    z_pred->data[0][0] = x->data[8][0];
    z_pred->rows = 1; z_pred->cols = 1;
}
void mag_H_jac(Matrix *H, const Matrix *x) {
    (void)x;
    mat_zero(H, 1, 9);
    H->data[0][8] = 1.0f;
}
void mag_R_init(Matrix *R, float sigma_yaw) {
    mat_zero(R, 1, 1);
    R->data[0][0] = sigma_yaw * sigma_yaw;
}
