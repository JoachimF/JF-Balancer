#ifndef _UART_CONSOLE_H_
#define _UART_CONSOLE_H_

#include <stdio.h>

/*extern void lancer_les_mesure(void) ;
extern void envoyer_donnees_mesure(void) ;
extern uint32_t RPM ;*/

void uart_console_task(void *pvParameters) ;

#endif