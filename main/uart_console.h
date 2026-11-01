#ifndef _UART_CONSOLE_H_
#define _UART_CONSOLE_H_

#include <stdio.h>

/*extern void lancer_les_mesure(void) ;
extern void envoyer_donnees_mesure(void) ;
extern uint32_t RPM ;*/

typedef struct
{
    /* data */
    uint16_t nb_tour_mesure ;
    uint16_t distance_plan_mm ;
    uint16_t rayon_test_A_mm ;
    uint16_t rayon_test_B_mm ;
    uint16_t nb_echantillons ;
    uint16_t rpm_mesure ;
    uint16_t rayon_correction_A_mm ;
    uint16_t rayon_correction_B_mm ;
}_param_t;

void uart_console_task(void *pvParameters) ;

#endif