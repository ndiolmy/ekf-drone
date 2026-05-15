#ifndef EKF_H
#define EKF_H

#include "matrix.h"

/*
 * Moteur EKF générique.
 * Les modèles dynamique et d'observation sont injectés via des
 * pointeurs de fonctions — ekf.c ne connaît pas le drone.
 *
 * Dimensions :
 *   n = taille du vecteur d'état  (9 pour le drone)
 *   m = taille du vecteur mesure  (variable selon le capteur)
 */
typedef struct {

    /* --- État et covariance --- */
    Matrix x;   /* vecteur d'état estimé        (n x 1) */
    Matrix P;   /* covariance d'erreur           (n x n) */

    /* --- Matrices de bruit --- */
    Matrix Q;   /* bruit de processus            (n x n) — réglage utilisateur */
    Matrix R;   /* bruit de mesure               (m x m) — réglage utilisateur */

    /* --- Modèle dynamique (prédiction) --- */
    /* f  : propagation d'état    x_new = f(x, u)           */
    void (*f)    (Matrix *x_out, const Matrix *x, const Matrix *u);
    /* F_jac : Jacobienne de f    F = df/dx évaluée en x    */
    void (*F_jac)(Matrix *F,     const Matrix *x, const Matrix *u);

    /* --- Modèle d'observation (correction) --- */
    /* h  : observation prédite   z_pred = h(x)             */
    void (*h)    (Matrix *z_pred, const Matrix *x);
    /* H_jac : Jacobienne de h    H = dh/dx évaluée en x    */
    void (*H_jac)(Matrix *H,      const Matrix *x);

} EKF;

/* --- API publique --- */

/* Initialise l'EKF avec un état x0 et une covariance P0 initiale */
void ekf_init(EKF *ekf, const Matrix *x0, const Matrix *P0);

/* Étape de prédiction — appelée à chaque tick IMU (~200 Hz)
 *   u : vecteur commande IMU [ax, ay, az, p, q, r] (6 x 1) */
void ekf_predict(EKF *ekf, const Matrix *u);

/* Étape de correction — appelée à chaque mesure capteur (~10 Hz)
 *   z : vecteur mesure brut (dimension dépend du capteur)  */
void ekf_update(EKF *ekf, const Matrix *z);

#endif 
