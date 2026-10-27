#include "uart_console.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "calibration.h"

extern void lancer_les_mesure(void) ;
extern void envoyer_donnees_mesure(void) ;
extern uint32_t RPM ;

#define UART_CONSOLE_NUM UART_NUM_0
#define BUF_SIZE (1024)

void uart_console_task(void *pvParameters)
{
    // 1. Configuration de l'UART 0 (Console standard)
    uart_config_t uart_config = {
        .baud_rate = 115200, // Ajustez selon la vitesse de votre moniteur
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // On configure et installe le driver (pas besoin de ré-allouer si déjà fait)
    uart_param_config(UART_CONSOLE_NUM, &uart_config);
    uart_driver_install(UART_CONSOLE_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    printf("\n--- Console Prete : tapez 'm' pour mesurer, 'd' pour dumper ---\n");

    while (1) {
        // Lecture non-bloquante ou avec un timeout court (20ms) pour laisser FreeRTOS respirer
        int len = uart_read_bytes(UART_CONSOLE_NUM, data, BUF_SIZE - 1, pdMS_TO_TICKS(20));
        
        if (len > 0) {
            // Analyser chaque caractère reçu
            for (int i = 0; i < len; i++) {
                char commande = data[i];
                
                if (commande == 'm' || commande == 'M') {
                    lancer_les_mesure() ;
                } 
                else if (commande == 'd' || commande == 'D') {
                    envoyer_donnees_mesure();
                }
                else if (commande == 'c' || commande == 'C') {
                    RPM = 0 ;
                }
                else if (commande == '0') cal_run0();
                else if (commande == 'a' || commande == 'A') cal_runA();
                else if (commande == 'b' || commande == 'B') cal_runB();
                else if (commande == 'k' || commande == 'K') cal_calcul();
                else if (commande == 'v' || commande == 'V') cal_verif();
            }
        }
        // Petit délai pour céder le processeur aux tâches prioritaires (ex: capture MPU)
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    free(data);
    vTaskDelete(NULL);
}