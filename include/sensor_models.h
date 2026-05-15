#ifndef SENSOR_MODELS_H
#define SENSOR_MODELS_H
 
#include "matrix.h"
 
/*
 * Modèles d'observation pour chaque capteur.
 * Chaque capteur expose :
 *   - une fonction h(x)     : observation prédite depuis l'état
 *   - une fonction H_jac(x) : Jacobienne dh/dx
 *   - une fonction R_init() : initialisation de la matrice de bruit R
 *
 * Ces fonctions sont passées en pointeurs à ekf_update().
 */
 
/* ------------------------------------------------------------------ */
/* GPS — mesure position 3D                                            */
/* Dimension mesure : 3  (x, y, z)                                    */
/* ------------------------------------------------------------------ */
#define GPS_DIM 3
 
void gps_h    (Matrix *z_pred, const Matrix *x);
void gps_H_jac(Matrix *H,      const Matrix *x);
 
/* Bruit GPS typique : sigma_pos = 2.5 m (CEP 50%)
 * Remplacer par les valeurs constructeur ou calibration terrain */
void gps_R_init(Matrix *R, float sigma_pos);
 
/* ------------------------------------------------------------------ */
/* Baromètre — mesure altitude                                         */
/* Dimension mesure : 1  (z)                                           */
/* ------------------------------------------------------------------ */
#define BARO_DIM 1
 
void baro_h    (Matrix *z_pred, const Matrix *x);
void baro_H_jac(Matrix *H,      const Matrix *x);
void baro_R_init(Matrix *R, float sigma_alt);
 
/* ------------------------------------------------------------------ */
/* Magnétomètre — mesure cap (yaw)                                     */
/* Dimension mesure : 1  (psi)                                         */
/* Note : modèle simplifié — suppose le drone approximativement plat   */
/* ------------------------------------------------------------------ */
#define MAG_DIM 1
 
void mag_h    (Matrix *z_pred, const Matrix *x);
void mag_H_jac(Matrix *H,      const Matrix *x);
void mag_R_init(Matrix *R, float sigma_yaw);
 
/* ------------------------------------------------------------------ */
/* Utilitaires communs                                                 */
/* ------------------------------------------------------------------ */
 
/* Normalise un angle dans [-pi, pi] */
float angle_wrap(float angle);
 
#endif 
 
