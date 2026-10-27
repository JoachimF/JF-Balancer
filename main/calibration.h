#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_

#include <stdio.h>
#include <complex.h>

#define CAL_RPM          3000     // vitesse unique de calibration et d'équilibrage
#define CAL_NB_MOYENNES  3        // acquisitions moyennées par run
#define MASSE_ESSAI_A_G  2.0      // masse d'essai plan A (g)
#define ANGLE_ESSAI_A    0.0      // angle par rapport au repère laser (deg)
#define MASSE_ESSAI_B_G  2.0
#define ANGLE_ESSAI_B    0.0

typedef double complex cplx ;

void cal_run0(void);
void cal_runA(void);
void cal_runB(void);
void cal_calcul(void);
void cal_verif(void);

#endif