#ifndef SIM_H
#define SIM_H

#include "matrix.h"

/*
 * Simulation d'une trajectoire drone et génération de mesures bruitées.
 * Utilisé dans main.c pour valider l'EKF sans matériel réel.
 */

/* Paramètres de simulation */
#define SIM_DT       0.005f    /* pas de temps (s) — doit égaler IMU_DT  */
#define SIM_DURATION 30.0f     /* durée totale de la simulation (s)       */
#define SIM_STEPS    ((int)(SIM_DURATION / SIM_DT))

typedef struct {
    float t;            /* temps courant (s)          */
    Matrix x_true;      /* état vrai (9 x 1)          */
    Matrix imu;         /* mesure IMU bruitée (6 x 1) */
    Matrix gps;         /* mesure GPS bruitée (3 x 1) */
    float  baro;        /* mesure baro bruitée (m)    */
    int    gps_ready;   /* 1 si une mesure GPS est disponible à ce pas */
} SimState;

/* Initialise la simulation à t=0 avec une position et attitude données */
void sim_init(SimState *s, const Matrix *x0);

/* Avance d'un pas de temps — met à jour x_true, génère les mesures bruitées */
void sim_step(SimState *s);

/* Niveaux de bruit (écart-type) — ajustables pour tester la robustesse */
typedef struct {
    float accel_std;   /* m/s²  — bruit accéléromètre  */
    float gyro_std;    /* rad/s — bruit gyroscope       */
    float gps_std;     /* m     — bruit position GPS    */
    float baro_std;    /* m     — bruit baromètre       */
} SimNoise;

extern SimNoise SIM_NOISE_DEFAULT;

#endif 
