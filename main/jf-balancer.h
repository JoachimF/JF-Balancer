#ifndef _JF_BALANCER_H_
#define _JF_BALANCER_H_

#include "filtres.h"

void calcul_des_resultats(void) ;
void envoyer_donnees_mesure(void) ;
void calcul_des_angles(data_mpu_t *data,uint16_t nb_mesure) ;
#endif