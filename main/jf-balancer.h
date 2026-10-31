#ifndef _JF_BALANCER_H_
#define _JF_BALANCER_H_

#include "filtres.h"
typedef struct
{
    /* data */
    uint16_t nb_tour_mesure ;
    uint16_t distance_plan_mm ;
    uint16_t diametre_A_mm ;
    uint16_t diametre_B_mm ;
    uint16_t nb_echantillons ;
    uint16_t rpm_mesure ;
    uint16_t rayon_masse_test_A_mm ;
    uint16_t rayon_masse_test_B_mm ;
}_param_t;


void calcul_des_resultats(void) ;
void envoyer_donnees_mesure(void) ;
void calcul_des_angles(data_mpu_t *data,uint16_t nb_mesure) ;
#endif