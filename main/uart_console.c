#include "uart_console.h"
#include <stdio.h>
#include <string.h>>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "calibration.h"
#include "jf-balancer.h"

extern void lancer_les_mesure(void) ;
extern void envoyer_donnees_mesure(void) ;
extern void mesure_a_vide(void) ;
extern uint32_t RPM ;
extern _param_t param ;

#define UART_CONSOLE_NUM UART_NUM_0
#define BUF_SIZE (1024)

static void traiter_parametre(char *cmd)
{
    if (cmd[0] != '$')
        return;

    if (strcmp(cmd, "$$") == 0)
    {
        printf("$1=%u\r\n", (unsigned int)param.nb_tour_mesure);
        printf("$2=%u\r\n", (unsigned int)param.distance_plan_mm);
        printf("$3=%u\r\n", (unsigned int)param.diametre_A_mm);
        printf("$4=%u\r\n", (unsigned int)param.diametre_B_mm);
        printf("$5=%u\r\n", (unsigned int)param.nb_echantillons);
        printf("$6=%u\r\n", (unsigned int)param.rpm_mesure);
        printf("$7=%u\r\n", (unsigned int)param.rayon_masse_test_A_mm);
        printf("$8=%u\r\n", (unsigned int)param.rayon_masse_test_B_mm);

        fflush(stdout);
        return;
    }
    char *end;
    long numero = strtol(cmd + 1, &end, 10);

    if (end == cmd + 1 || *end != '=')
        return;

    char *valeur_txt = end + 1;
    long valeur = strtol(valeur_txt, &end, 10);

    if (end == valeur_txt || *end != '\0')
        return;

    if (valeur < 0 || valeur > UINT16_MAX)
        return;
    printf("Paramtere %ld = %u\n",numero,(uint16_t)valeur) ;
    switch (numero)
    {
        case 1: param.nb_tour_mesure = (uint16_t)valeur; break;
        case 2: param.distance_plan_mm = (uint16_t)valeur; break;
        case 3: param.diametre_A_mm = (uint16_t)valeur; break;
        case 4: param.diametre_B_mm = (uint16_t)valeur; break;
        case 5: param.nb_echantillons = (uint16_t)valeur; break;
        case 6: param.rpm_mesure = (uint16_t)valeur; break;
        case 7: param.rayon_masse_test_A_mm = (uint16_t)valeur; break;
        case 8: param.rayon_masse_test_B_mm = (uint16_t)valeur; break;

        default: printf("Paramtere inconnu\n") ;
        break;
    }
}

static void traiter_commande_simple(char commande)
{
    switch (commande)
    {
        case 'm':
        case 'M':
            lancer_les_mesure();
            break;

        case 'd':
        case 'D':
            envoyer_donnees_mesure();
            break;

        case 'c':
        case 'C':
            RPM = 0;
            break;

        case '0':
            cal_run0();
            break;

        case 'a':
        case 'A':
            cal_runA();
            break;

        case 'b':
        case 'B':
            cal_runB();
            break;

        case 'k':
        case 'K':
            cal_calcul();
            break;

        case 'v':
        case 'V':
            cal_verif();
            break;

        case 't':
        case 'T':
            mesure_a_vide();
            break;

        default:
            printf("Commande inconnue : %c\n", commande);
            break;
    }
}

static void traiter_ligne(char *ligne)
{
    if (ligne[0] == '\0') return;

    if (ligne[0] == '$')
    {
        traiter_parametre(ligne);
    }
    else
    {
        traiter_commande_simple(ligne[0]);
    }
}

void uart_console_task(void *pvParameters)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_CONSOLE_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_CONSOLE_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));

    uint8_t c;
    char ligne[64];
    size_t index = 0;

    printf("\r\n--- Console prête ---\r\n");
    fflush(stdout);

    while (1)
    {
        int len = uart_read_bytes(
            UART_CONSOLE_NUM,
            &c,
            1,
            portMAX_DELAY
        );

        if (len != 1)
            continue;

        // Fin de ligne CR = 0x0D
        if (c == 0x0D)
        {
            printf("\r\n");
            ligne[index] = '\0';

            if (index > 0)
            {
                printf("Reçu : %s\r\n", ligne);
                traiter_ligne(ligne);
            }

            index = 0;
            fflush(stdout);
        }
        else
        {
            // Echo immédiat
            putchar(c);
            fflush(stdout);

            if (index < sizeof(ligne) - 1)
            {
                ligne[index++] = (char)c;
            }
            else
            {
                index = 0;
                printf("\r\nErreur : ligne trop longue\r\n");
                fflush(stdout);
            }
        }
    }
}

/*void uart_console_task(void *pvParameters)
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
                else if (commande == 't' || commande == 'T') mesure_a_vide();                     
            }
        }
        // Petit délai pour céder le processeur aux tâches prioritaires (ex: capture MPU)
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    free(data);
    vTaskDelete(NULL);
}*/