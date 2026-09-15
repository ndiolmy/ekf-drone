#ifndef IMU_MODEL_H
#define IMU_MODEL_H

#include "matrix.h"

/*
 * Modèle dynamique du drone basé sur les mesures IMU.
 *
 * Vecteur d'état x (9 x 1) :
 *   [0] x   position Nord    (m)
 *   [1] y   position Est     (m)
 *   [2] z   altitude         (m)
 *   [3] vx  vitesse Nord     (m/s)
 *   [4] vy  vitesse Est      (m/s)
 *   [5] vz  vitesse verticale(m/s)
 *   [6] phi   roulis (roll)  (rad)
 *   [7] theta tangage(pitch) (rad)
 *   [8] psi   lacet  (yaw)   (rad)
 *
 * Vecteur commande u (6 x 1) :
 *   [0] ax  accélération x   (m/s²)
 *   [1] ay  accélération y   (m/s²)
 *   [2] az  accélération z   (m/s²)
 *   [3] p   vitesse angulaire x (rad/s)
 *   [4] q   vitesse angulaire y (rad/s)
 *   [5] r   vitesse angulaire z (rad/s)
 */

/* Pas de temps de la prédiction (secondes) — doit correspondre à la freq IMU */
#define IMU_DT 0.005f   /* 200 Hz */

/* Accélération gravitationnelle */
#define GRAVITY 9.81f

/* Propagation d'état : x_new = f(x, u)
 * Intègre les équations de la mécanique du vol sur un pas dt */
void imu_f(Matrix *x_out, const Matrix *x, const Matrix *u);

/* Jacobienne du modèle dynamique : F = df/dx évaluée en (x, u)
 * Matrice 9x9 calculée analytiquement */
//void imu_F_jac(Matrix *F, const Matrix *x, const Matrix *u);

/* Jacobienne par différences finies — alternative numérique à imu_F_jac
 * Plus lente mais utile pour valider la version analytique */
void imu_F_jac_numerical(Matrix *F, const Matrix *x, const Matrix *u, float eps);

#endif 
