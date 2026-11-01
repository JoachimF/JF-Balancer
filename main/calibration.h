#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_

#include <stdio.h>
#include <stdbool.h>
#include <complex.h>
#include "filtres.h"
#include "uart_console.h"      // pour _param_t

#define CAL_RPM          6000    // vitesse unique de calibration et d'équilibrage
#define CAL_NB_MOYENNES  5        // acquisitions moyennées par mesure
#define MASSE_ESSAI_A_G  0.05     // masse d'essai plan A (g)
#define ANGLE_ESSAI_A    0.0      // angle de la masse d'essai / repère laser (deg)
#define MASSE_ESSAI_B_G  0.05
#define ANGLE_ESSAI_B    0.0

typedef double complex cplx;

// Résultat pour un plan
typedef struct
{
    double balourd;       // module du balourd réel (g.mm)
    double angle;         // position angulaire du balourd réel (deg) : côté "lourd"
    double masse;         // masse à AJOUTER (g) au rayon de correction
    double angle_cor;     // angle où AJOUTER cette masse (deg) = angle + 180
} balourd_t;

typedef struct
{
    cplx cal_V0[2];          // réponse capteurs 1 et 2, rotor seul
    cplx cal_VA[2];          // avec la masse d'essai dans le plan A
    cplx cal_VB[2];          // avec la masse d'essai dans le plan B
    cplx a11, a12, a21, a22; // matrice d'influence (V par g.mm)
    cplx det;
    double conditionnement;
    bool calibre;            // vrai quand la matrice est valide
} calibration_data_t;

// Calibration : étapes 0, a, b, k (voir uart_console)
void cal_run0(void);
void cal_runA(void);
void cal_runB(void);
void cal_calcul(const _param_t *param);

// Chargement d'une calibration déjà mesurée (6 couples {amplitude, phase en deg})
// ordre : V1_0, V2_0, V1_A, V2_A, V1_B, V2_B
void cal_charger(const double ref[6][2], const _param_t *param);
void init_calibration(const _param_t *param);

// *** Balourd ***
// Calcul pur, sans mesure : à partir de V1, V2 complexes (amplitude + phase du lockin)
int  cal_calcul_balourd(cplx V1, cplx V2, const _param_t *param,
                        balourd_t *balA, balourd_t *balB);
// Mesure à CAL_RPM, calcule et affiche le balourd et la correction
void cal_mesure_balourd(const _param_t *param);

void cal_verif(void);

#endif