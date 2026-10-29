#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_

#include <stdio.h>
#include <complex.h>

#define CAL_RPM          5000    // vitesse unique de calibration et d'équilibrage
#define CAL_NB_MOYENNES  5        // acquisitions moyennées par run
#define MASSE_ESSAI_A_G  0.1      // masse d'essai plan A (g)
#define ANGLE_ESSAI_A    0.0      // angle par rapport au repère laser (deg)
#define MASSE_ESSAI_B_G  0.1
#define ANGLE_ESSAI_B    0.0

#define DIAM_A_MM        66.0     // diamètre où tu poses les masses, plan A
#define DIAM_B_MM        40.0     // idem plan B
#define RAYON_A_MM       (DIAM_A_MM / 2.0)
#define RAYON_B_MM       (DIAM_B_MM / 2.0)

typedef double complex cplx ;

void cal_run0(void);
void cal_runA(void);
void cal_runB(void);
void cal_calcul(void);
void cal_verif(void);
static double masse_pour_diametre(cplx U, double diam_mm) ;

#endif