#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include "matrix.h"
#include "ekf.h"
#include "imu_model.h"
#include "sensor_models.h"
#include "sim.h"

/* ------------------------------------------------------------------ */
/* Gaussian noise generator (Box-Muller)                          */
/* ------------------------------------------------------------------ */
static float randn(float std) {
    float u1 = (float)(rand() + 1) / ((float)RAND_MAX + 1.0f);
    float u2 = (float)(rand() + 1) / ((float)RAND_MAX + 1.0f);
    return std * sqrtf(-2.0f * logf(u1)) * cosf(2.0f * (float)M_PI * u2);
}

/* ------------------------------------------------------------------ */
/* True trajectory : circle at constant altitude         */
/* ------------------------------------------------------------------ */
static void true_state(float t, Matrix *x_true) {
    float omega = 0.2f;   /* rad/s — circle's angular velocity */
    float R     = 10.0f;  /* m     — rayon */

    mat_zero(x_true, 9, 1);
    x_true->data[0][0] = R * cosf(omega * t);          /* px */
    x_true->data[1][0] = R * sinf(omega * t);          /* py */
    x_true->data[2][0] = 5.0f;                         /* pz — constant altitude  */
    x_true->data[3][0] = -R * omega * sinf(omega * t); /* vx */
    x_true->data[4][0] =  R * omega * cosf(omega * t); /* vy */
    x_true->data[5][0] = 0.0f;                         /* vz */
    x_true->data[6][0] = 0.0f;                         /* phi */
    x_true->data[7][0] = 0.0f;                         /* theta */
    x_true->data[8][0] = omega * t;                    /* psi */
}

/* ------------------------------------------------------------------ */
/* Main                                                                */
/* ------------------------------------------------------------------ */
int main(void) {
    srand((unsigned)time(NULL));

    /* --- EKF Initialization --- */
    EKF ekf;

    /* Models plugging */
    ekf.f     = imu_f;
    ekf.F_jac = imu_F_jac_numerical;  /* for now finite differences */
    ekf.h     = gps_h;
    ekf.H_jac = gps_H_jac;

	float eps = 1e-4f;
    /* Initial state — start from the true position with a little error */
    Matrix x0, P0;
    true_state(0.0f, &x0);
    x0.data[0][0] += randn(1.0f);  /* position initial error */
    x0.data[1][0] += randn(1.0f);

    /* P0 — initial uncertainty */
    mat_eye(&P0, 9);
    for (int i = 0; i < 9; i++)
        P0.data[i][i] = (i < 3) ? 5.0f : 1.0f;  /* more uncertainties on position */

    /* Q — process noise */
    mat_zero(&ekf.Q, 9, 9);
    for (int i = 3; i < 6; i++) ekf.Q.data[i][i] = 0.1f;   /* velocities */
    for (int i = 6; i < 9; i++) ekf.Q.data[i][i] = 0.01f;  /* angles */

    /* R — GPS noise*/
    gps_R_init(&ekf.R, 2.5f);  /* sigma_pos = 2.5 m */

    ekf_init(&ekf, &x0, &P0);

    /* --- Output for Python plot --- */
    FILE *f = fopen("results.csv", "w");
    if (!f) { fprintf(stderr, "Impossible to open results.csv\n"); return 1; }
    fprintf(f, "t,px_true,py_true,pz_true,px_est,py_est,pz_est,px_gps,py_gps,pz_gps\n");

    /* --- Simulation loop --- */
    int gps_rate = 20;  /* 1 GPS correction every 20 predictions = 10 Hz */

    for (int k = 0; k < SIM_STEPS; k++) {
        float t = k * IMU_DT;

        /* true state at t and t+dt to calculate the simulated IMU */
        Matrix x_true, x_next;
        true_state(t,           &x_true);
        true_state(t + IMU_DT,  &x_next);

        /* --- simulated IMU command (accelerations + angular velocities with noise) --- */
        Matrix u;
        mat_zero(&u, 6, 1);
        /* accelerations : numerical differenciation of velocities + noise */
        u.data[0][0] = (x_next.data[3][0] - x_true.data[3][0]) / IMU_DT + randn(0.1f);
        u.data[1][0] = (x_next.data[4][0] - x_true.data[4][0]) / IMU_DT + randn(0.1f);
        u.data[2][0] = (x_next.data[5][0] - x_true.data[5][0]) / IMU_DT + GRAVITY + randn(0.05f);
        /* angular velocities + noise */
        u.data[3][0] = randn(0.01f);  /* p — roll velocity */
        u.data[4][0] = randn(0.01f);  /* q - pitch velocity*/
        u.data[5][0] = 0.2f + randn(0.01f);  /* r — yaw velocity */

        /* --- IMU prediction (~200 Hz) --- */
        ekf_predict(&ekf, &u,eps);

        /* --- GPS correction (~10 Hz) --- */
        float px_gps = 0.0f, py_gps = 0.0f, pz_gps = 0.0f;
        if (k % gps_rate == 0) {
            Matrix z_gps;
            z_gps.rows = 3; z_gps.cols = 1;
            z_gps.data[0][0] = x_true.data[0][0] + randn(2.5f);
            z_gps.data[1][0] = x_true.data[1][0] + randn(2.5f);
            z_gps.data[2][0] = x_true.data[2][0] + randn(2.5f);

            ekf.h     = gps_h;
            ekf.H_jac = gps_H_jac;
            ekf_update(&ekf, &z_gps);

            px_gps = z_gps.data[0][0];
            py_gps = z_gps.data[1][0];
            pz_gps = z_gps.data[2][0];
        }

        if (k % 10 == 0) {
            fprintf(f, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                t,
                x_true.data[0][0], x_true.data[1][0], x_true.data[2][0],
                ekf.x.data[0][0],  ekf.x.data[1][0],  ekf.x.data[2][0],
                px_gps, py_gps, pz_gps);
        }
    }

    fclose(f);
    printf("Simulation ended — results dans results.csv\n");
    printf("run : python3 plot_results.py\n");
    return 0;
}
