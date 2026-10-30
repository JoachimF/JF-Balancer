/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "jf-balancer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <mpu6050.h>
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "math.h"
#include "esp_dsp.h"
#include "driver/uart.h"
#include "servo.h"
#include "hardware.h"
#include "filtres.h"
#include "uart_console.h"
#include <complex.h>
#include "calibration.h"
//#include "esp_psram.h"

#define MPU60501_INT_PIN 19
#define MPU60502_INT_PIN 33
#define LASER_INT_PIN 4

//#define SAMPLES 64
#define RPM_ERROR 50
#define STABILIZATION_TIME 7000

#define GYRO2 1

#define EVT_START1  (1 << 0)

servo_t esc ;
uint32_t RPM ;
uint32_t ppm ;
_param_t param = {
    .nb_tour_mesure = 10,
    .distance_plan_mm = 100,
    .diametre_A_mm = 70,
    .diametre_B_mm = 70,
    .nb_echantillons = 2048,
    .rpm_mesure = 3000,
    .rayon_masse_test_A_mm = 35,
    .rayon_masse_test_B_mm = 35
};

bool rpm_rdy ;
static volatile bool mesure_encours = false, mesure_encours2 = false ;
bool START_MESURE =false ;
uint16_t mesure ;

float data_x1[2*SAMPLES] ;
float data_y1[2*SAMPLES] ;
float data_x2[2*SAMPLES] ;
float data_y2[2*SAMPLES] ;

data_mpu_t raw_data1[SAMPLES] ;
data_mpu_t raw_data2[SAMPLES] ;

data_output_t data_output[5] ;

TaskHandle_t read_accel ;
TaskHandle_t read_accel2 ;
TaskHandle_t regul_rpm ;

extern cplx last_V1, last_V2;

static EventGroupHandle_t acq_evt;

bool is_first = 1 ;
//static volatile uint64_t start_time ;
//static volatile uint64_t start_time2 ;
static volatile uint32_t freq_laser ;
static volatile uint64_t period ;
static volatile uint32_t laser_rpm ;
mpu6050_dev_t dev1 = { 0 };
mpu6050_dev_t dev2 = { 0 };
uint32_t nb_sample = 0, nb_sample2 = 0 ;
//bool get_data = 0 ;


static const char *TAG = "example";

mpu6050_acceleration_t accel = { 0 };
mpu6050_acceleration_t accel2 = { 0 };

uint64_t current_time ;
uint64_t current_time2 ;
uint64_t laser_isr_last ;

static volatile uint64_t laser_first = 0, laser_last = 0 ;
static volatile int32_t laser_count = 10 ;

static void IRAM_ATTR mpu6050_isr(void* arg)
{
    BaseType_t woken = pdFALSE;
    current_time = esp_timer_get_time() ;
    if(mesure_encours == true)
    {
        vTaskNotifyGiveFromISR(read_accel, &woken);
        portYIELD_FROM_ISR(woken);
    }

    //xEventGroupSetBitsFromISR(data_rdy, EVT_ACQ_DONE, &woken);
    //if (woken) portYIELD_FROM_ISR();
}

static void IRAM_ATTR mpu60502_isr(void* arg)
{
    BaseType_t woken = pdFALSE;
    current_time2 = esp_timer_get_time() ;
    if(mesure_encours2 == true)
    {
        vTaskNotifyGiveFromISR(read_accel2, &woken);
        portYIELD_FROM_ISR(woken);
    }

    //xEventGroupSetBitsFromISR(data_rdy, EVT_ACQ_DONE2, &woken);
    //if (woken) portYIELD_FROM_ISR();
}

static uint64_t ts1[SAMPLES], ts2[SAMPLES];
static volatile uint32_t nb_isr1, nb_isr2;

static void IRAM_ATTR laser_isr(void *arg)
{
/*    uint64_t now = esp_timer_get_time();
    
    if (laser_count == 0)
    {
        laser_first = now;
        nb_sample = 0 ;
        nb_sample2 = 0 ;
        mesure_encours = true ;
        mesure_encours2 = true ;
        xEventGroupSetBits(acq_evt, EVT_START1);
    }
    period = now - laser_isr_last ;
    if(period > 0) freq_laser = 1000000 / period ;
    laser_rpm = freq_laser * 60 ;
    laser_isr_last  = now;    
    if(mesure_encours == true || mesure_encours2 == true)
    {
        laser_count++;
        laser_last = now;
    }*/
    uint64_t now = esp_timer_get_time();
    period = now - laser_isr_last ;
    if(period > 0) freq_laser = 1000000 / period ;
    laser_rpm = freq_laser * 60 ;
    laser_isr_last  = now;   
    
    if (laser_count == 0)
    {
        laser_first = now;
        nb_sample = nb_sample2 = 0;
        nb_isr1 = nb_isr2 = 0;
        mesure_encours = mesure_encours2 = true;      // plus de xEventGroupSetBits
    }
    if (mesure_encours && nb_isr1 < SAMPLES) {
        ts1[nb_isr1++] = esp_timer_get_time();       // date exacte de l'interruption
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(read_accel, &woken);
        portYIELD_FROM_ISR(woken);
    }
    if (mesure_encours && nb_isr2 < SAMPLES) {
        ts2[nb_isr2++] = esp_timer_get_time();       // date exacte de l'interruption
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(read_accel2, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

void mesure_a_vide(void)
{
    printf("\nMesures à vide\n");
    //vTaskResume(read_accel);
    //vTaskResume(read_accel2);
    laser_count = 0 ;
    uint64_t now = esp_timer_get_time();
    laser_first = now ;
    START_MESURE = true ;
    nb_sample = nb_sample2 = 0;
    nb_isr1 = nb_isr2 = 0;
    mesure_encours = mesure_encours2 = true; 
    //xEventGroupSetBits(acq_evt, EVT_START1);
    // Mettre votre flag de capture à true ou appeler votre routine de mesure ici
    //vTaskDelay(pdMS_TO_TICKS(1000)); 
    while(mesure_encours || mesure_encours2) 
    {
        if(nb_sample >= SAMPLES) mesure_encours = false ;
        if(nb_sample2 >= SAMPLES) mesure_encours2 = false ;
        printf(".") ;
        vTaskDelay(pdMS_TO_TICKS(10)) ; 
    }
    
    vTaskDelay(pdMS_TO_TICKS(100)); 
    envoyer_donnees_mesure() ;
}
// Remplacer ces fonctions par votre logique réelle
int declencher_mesure(uint16_t c_rpm) {

    xTaskNotifyStateClear(read_accel);
    xTaskNotifyStateClear(read_accel2);
    ulTaskNotifyValueClear(read_accel, 0xFFFFFFFF);
    ulTaskNotifyValueClear(read_accel2, 0xFFFFFFFF);

    //ESP_ERROR_CHECK(gpio_intr_enable(LASER_INT_PIN)) ; 
    printf("\n[Console] Lancement de la mesure à %drpm...\n",c_rpm);
    RPM = c_rpm ;
    vTaskDelay(pdMS_TO_TICKS(1000));
    while(!rpm_rdy) vTaskDelay(pdMS_TO_TICKS(100)); ;
    printf("\nRPM stabilisés à %ldrpm...\n",laser_rpm);
    //vTaskResume(read_accel);
    //vTaskResume(read_accel2);
    laser_count = 0 ;
    START_MESURE = true ;
    vTaskDelay(pdMS_TO_TICKS(1000)); 
    // Mettre votre flag de capture à true ou appeler votre routine de mesure ici
    while(mesure_encours || mesure_encours2) 
    {
        if(nb_sample >= SAMPLES) mesure_encours = false ;
        if(nb_sample2 >= SAMPLES) mesure_encours2 = false ;
        printf(".") ;
        vTaskDelay(pdMS_TO_TICKS(10)) ; 
    }
    
    vTaskDelay(pdMS_TO_TICKS(100)); 
    envoyer_donnees_mesure() ;
    calcul_des_resultats() ;
    return true ;
    //ESP_ERROR_CHECK(gpio_intr_enable(MPU60501_INT_PIN)) ;
    //ESP_ERROR_CHECK(gpio_intr_enable(MPU60502_INT_PIN)) ;

}

void envoyer_donnees_mesure(void) {
    printf("\n[Console] Envoi des donnees de l'accelero...\n");

    printf("###ACCEL1-%d\n",mesure+1) ;
    printf("t;Y\n") ;
    for(int j=0;j<SAMPLES;j++)
    {
        printf("%.0f;%f\n",raw_data1[j].time,raw_data1[j].gyroy) ;
    }
    printf("###\n") ;

    printf("###ACCEL2-%d\n",mesure+1) ;
    printf("t;Y\n") ;
    for(int j=0;j<SAMPLES;j++)
    {
        printf("%.0f;%f\n",raw_data2[j].time,raw_data2[j].gyroy) ;
    }
    printf("###\n") ;
}

void lancer_les_mesure(void)
{
    uint16_t rpms[] = {4000,4500,5000,5500,6000} ;
    mesure = 0 ;
    printf("###sample=%d###\n",SAMPLES);
    while(mesure < 5)
    {
        declencher_mesure(rpms[mesure]) ;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    RPM = 0 ;
}



void task_lecture_accel2()
{
    float accely ;
    gpio_config_t io_conf_mpu60502_isr = {
        .pin_bit_mask = (1ULL << MPU60502_INT_PIN), // Sélection de la broche
        .mode = GPIO_MODE_INPUT,                   // Mode Entrée
        .pull_up_en = GPIO_PULLUP_ENABLE,          // Active la résistance de Pull-up interne
        .pull_down_en = GPIO_PULLDOWN_DISABLE,      // Désactive la résistance de Pull-down
        .intr_type = GPIO_INTR_POSEDGE             // Déclenchement sur Flanc Descendant (Falling Edge)
    };
    ESP_LOGI(TAG, "Config GPIO 33") ;
    ESP_ERROR_CHECK(gpio_config(&io_conf_mpu60502_isr));

    ESP_ERROR_CHECK(gpio_isr_handler_add(MPU60502_INT_PIN, mpu60502_isr, (void*) MPU60502_INT_PIN));
    ESP_ERROR_CHECK(gpio_intr_enable(MPU60502_INT_PIN)) ;
    ESP_LOGI(TAG, "Config GPIO 12") ;
    
    ESP_LOGI(TAG, "Accel 2 TASK OK") ;
    while(1) {
        //xEventGroupWaitBits(acq_evt, EVT_START1, pdTRUE, pdFALSE, portMAX_DELAY);
        //while(nb_sample2 < SAMPLES){
           /* ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            ESP_ERROR_CHECK(mpu6050_get_acceleration(&dev2, &accel2));
            raw_data2[nb_sample2].gyroy = accel2.y ;
            raw_data2[nb_sample2].time = (float)current_time2 - (float)laser_first ; 
            nb_sample2++ ;*/
            ulTaskNotifyTake(pdFALSE, portMAX_DELAY);   // pdFALSE : une notification = un échantillon
            //ESP_ERROR_CHECK(mpu6050_get_acceleration(&dev2, &accel2));
            if(mesure_encours2)
            {
                ESP_ERROR_CHECK(mpu6050_get_acceleration_axis(&dev2,MPU6050_Y_AXIS, &accely)) ;
                raw_data2[nb_sample2].gyroy = -accely;
                //raw_data2[nb_sample2].gyroy = -accel2.y;
                raw_data2[nb_sample2].time = (float)current_time2 - (float)laser_first ;
                if (++nb_sample2 >= SAMPLES) mesure_encours2 = false;
            }

        //}   
        //mesure_encours2 = false ; 
        //printf("Fin de l'acquisition MPU2\n") ;
    }
}

void task_lecture_accel1()
{
    float accely ;
    gpio_config_t io_conf_mpu6050_isr = {
        .pin_bit_mask = (1ULL << MPU60501_INT_PIN), // Sélection de la broche
        .mode = GPIO_MODE_INPUT,                   // Mode Entrée
        .pull_up_en = GPIO_PULLUP_ENABLE,          // Active la résistance de Pull-up interne
        .pull_down_en = GPIO_PULLDOWN_DISABLE,      // Désactive la résistance de Pull-down
        .intr_type = GPIO_INTR_POSEDGE             // Déclenchement sur Flanc Descendant (Falling Edge)
    };
    ESP_LOGI(TAG, "Config GPIO 19") ;
    ESP_ERROR_CHECK(gpio_config(&io_conf_mpu6050_isr));

    ESP_LOGI(TAG, "Config handler mpu") ;
    ESP_ERROR_CHECK(gpio_isr_handler_add(MPU60501_INT_PIN, mpu6050_isr, (void*) MPU60501_INT_PIN));
    ESP_ERROR_CHECK(gpio_intr_enable(MPU60501_INT_PIN)) ;

    ESP_LOGI(TAG, "Accel 1 TASK OK") ;
    while(1) {
        //xEventGroupWaitBits(acq_evt, EVT_START1, pdTRUE, pdFALSE, portMAX_DELAY);
        //while(nb_sample < SAMPLES){
            /*ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            ESP_ERROR_CHECK(mpu6050_get_acceleration(&dev1, &accel));

            raw_data1[nb_sample].gyroy = -accel.y ;
            raw_data1[nb_sample].time = (float)current_time - (float)laser_first ; 
            //printf("%.0f;%f\n",raw_data1[nb_sample].time,raw_data1[nb_sample].gyroy) ;
            #ifndef GYRO2
                nb_sample2 = nb_sample ;
                raw_data2[nb_sample].time = (float)current_time - (float)start_time ;
                raw_data2[nb_sample].gyroy = accel.y / 10 ;
                nb_sample2++ ;
            #endif
            nb_sample++ ;*/
            ulTaskNotifyTake(pdFALSE, portMAX_DELAY);   // pdFALSE : une notification = un échantillon
            //ESP_ERROR_CHECK(mpu6050_get_acceleration(&dev1, &accel));
            if(mesure_encours)
            {
                ESP_ERROR_CHECK(mpu6050_get_acceleration_axis(&dev1,MPU6050_Y_AXIS, &accely)) ;
                raw_data1[nb_sample].gyroy = -accely;
                //raw_data1[nb_sample].gyroy = -accel.y;
                raw_data1[nb_sample].time = (float)current_time - (float)laser_first ;
                if (++nb_sample >= SAMPLES) mesure_encours = false;
            }
            //if (++nb_sample >= SAMPLES) mesure_encours = false;
        //}
        //mesure_encours = false ; 
        //printf("Fin de l'acquisition MPU1\n") ;
    }
}

void calcul_des_resultats(void)
{
        //while(nb_sample2 < SAMPLES) vTaskDelay(80 / portTICK_PERIOD_MS); // Attente de la fin des mesure du MPU2
        //envoyer_donnees_mesure() ;
        double f_ref = 0 ;
        double t_laser0 = 0 ;
        static float sig[SAMPLES] ;
        static uint32_t tus[SAMPLES] ;
        float angle_corrige, amp_corrigee ;

        //Préparation des tableau pour la FFT
        printf("Préparation de la FFT 1\n") ;
        for(int i=0;i<SAMPLES;i++)
        {
            data_x1[i*2]     = raw_data1[i].time ;
            data_y1[i*2]     = raw_data1[i].time ;
            data_x1[1+2*i]   = raw_data1[i].gyrox ;
            data_y1[1+2*i]   = raw_data1[i].gyroy ;
        }
        //Préparation des tableau pour la FFT
        printf("Préparation de la FFT 2") ;
        for(int i=0;i<SAMPLES;i++)
        {
            data_x2[i*2]     = raw_data2[i].time ;
            data_y2[i*2]     = raw_data2[i].time ;
            data_x2[1+2*i]   = raw_data2[i].gyrox ;
            data_y2[1+2*i]   = raw_data2[i].gyroy ;
        }
        //printf("t;X;Y;t\n") ;
        //for(int j=0;j<SAMPLES*2;j+=2)
        //{
            //printf("%.0f;%f;%f\n",data_x[j],data_x[j+1],data_y[j+1]) ;
            //printf("%.0f;%f\n",data_x[j],data_y[j+1]) ;
        //}
        float dtmax = 0;
        for (int i = 1; i < SAMPLES; i++) {
            float d = fabsf(raw_data1[i].time - raw_data1[i-1].time - 1000.0f);
            if (d > dtmax) dtmax = d;
        }
        printf("plan A : ecart max a 1 ms = %.0f us\n", dtmax);

        float dtmax2 = 0;
        for (int i = 1; i < SAMPLES; i++) {
            float d = fabsf(raw_data2[i].time - raw_data2[i-1].time - 1000.0f);
            if (d > dtmax2) dtmax2 = d;
        }
        printf("plan B : ecart max a 1 ms = %.0f us\n", dtmax2);
#define FFT 1
#ifdef FFT    
        //compute_fft(raw_data1, SAMPLES);
        //compute_fft(raw_data2, SAMPLES);
#endif
        
        /*********Lockin MPU 1 *********************/
        for (int i = 0; i < SAMPLES; i++) {
            tus[i] = raw_data1[i].time;
            sig[i] = raw_data1[i].gyroy;      // ou gyrox, etc.
        }
        
        printf("laser count = %ld\n",laser_count) ;
        f_ref = (laser_count - 1) * 1e6 / (double)(laser_last - laser_first);
        t_laser0 = ((double)((int64_t)laser_first - (int64_t)laser_first)) * 1e-6; // en s, base de temps des échantillons

        lockin_result_t r = lockin(tus, sig, SAMPLES, f_ref, t_laser0); 
        angle_corrige = obtenir_angle_corrige1(r.phase_deg,f_ref) ;
        amp_corrigee = obtenir_amplitude_corrige1(r.amp,f_ref) ;

        // après le lockin MPU1
        last_V1 = r.X + I * r.Y;
        // après le lockin MPU2
        
        /*data_output[mesure].f_ref = f_ref ;
        data_output[mesure].amplitude = amp_corrigee ; //r.amp ;
        data_output[mesure].phase = r.phase_deg ;
        data_output[mesure].deg = r.n_used ; 
        data_output[mesure].angle = angle_corrige ;*/
        

        //printf("Laser count : %ld | f_ref = %.2f Hz | amplitude = %.4f | phase = %.1f deg (n=%d) | angle : %.1f\n",laser_count,f_ref, r.amp, r.phase_deg, r.n_used,angle_corrige);
        
        printf("###LOCKIN1-%d\n",mesure+1) ;
        printf("%f;%f;%f;%f;%d;%f\n",f_ref,r.amp,amp_corrigee,r.phase_deg,r.n_used,angle_corrige) ;
        printf("###\n") ;

         /*********Lockin MPU 2 *********************/
        for (int i = 0; i < SAMPLES; i++) {
            tus[i] = raw_data2[i].time;
            sig[i] = raw_data2[i].gyroy;      // ou gyrox, etc.
        }

        printf("laser count = %ld\n",laser_count) ;
        f_ref = (laser_count - 1) * 1e6 / (double)(laser_last - laser_first);
        t_laser0 = ((double)((int64_t)laser_first - (int64_t)laser_first)) * 1e-6; // en s, base de temps des échantillons    
        r = lockin(tus, sig, SAMPLES, f_ref, t_laser0); 
        angle_corrige = obtenir_angle_corrige2(r.phase_deg,f_ref) ;
        amp_corrigee = obtenir_amplitude_corrige2(r.amp,f_ref) ;

        last_V2 = r.X + I * r.Y;

        /*data_output[mesure].f_ref = f_ref ;
        data_output[mesure].amplitude = amp_corrigee ; //r.amp ;
        data_output[mesure].phase = r.phase_deg ;
        data_output[mesure].deg = r.n_used ; 
        data_output[mesure].angle = angle_corrige ;*/

        //printf("Laser count : %ld | f_ref = %.2f Hz | amplitude = %.4f | phase = %.1f deg (n=%d) | angle : %.1f\n",laser_count,f_ref, r.amp, r.phase_deg, r.n_used,angle_corrige);
        
        printf("###LOCKIN2-%d\n",mesure+1) ;
        printf("%f;%f;%f;%f;%d;%f\n",f_ref,r.amp,amp_corrigee,r.phase_deg,r.n_used,angle_corrige) ;
        printf("###\n") ;
        /******* Fin de l'envoi des données *******/
        mesure++;
}

void task_regul_rpm()
{
    ppm = 1000 ;
    uint16_t rpm_ok = 0 ;
    
    ESP_ERROR_CHECK(servo_set_pulse_us(&esc,ppm)) ;
    while(1)
    {
        rpm_rdy = false ;
        if(RPM > 500)
        {
            if(laser_rpm < (RPM - RPM_ERROR) || laser_rpm > (RPM + RPM_ERROR))
            {
                if(laser_rpm < RPM)
                    ppm++ ;
                else
                    ppm--;
                ESP_ERROR_CHECK(servo_set_pulse_us(&esc,ppm)) ;
                if(rpm_ok > 0)
                    rpm_ok-- ;
            } else {
                if(rpm_ok < 51)
                    rpm_ok++ ;
            }
            //ESP_ERROR_CHECK(servo_set_pulse_us(&esc,ppm)) ;
        }else{
            ppm = 1000 ;
            rpm_ok = 0 ;
            ESP_ERROR_CHECK(servo_set_pulse_us(&esc,0)) ;
            //vTaskDelay(50 / portTICK_PERIOD_MS); 
        }
        if(rpm_ok > 50)
            rpm_rdy = true ;
        else
            rpm_rdy = false ;
        vTaskDelay(50 / portTICK_PERIOD_MS); 
    }
    if((esp_timer_get_time() - laser_last) > 10000000)
        laser_rpm = 0 ;
}

void task(void *ignore)
{
    i2c_dev_t dev = { 0 };
    dev.port = I2C_NUM_1 ;
    dev.sda_pin = 25 ;
    dev.scl_pin = 26 ;
    dev.cfg.sda_io_num = 25 ;//21 ;
    dev.cfg.scl_io_num = 26; //22 ;
    dev.cfg.sda_pullup_en = GPIO_PULLUP_ENABLE ;
    dev.cfg.scl_pullup_en = GPIO_PULLUP_ENABLE ;

#if HELPER_TARGET_IS_ESP32
    dev.cfg.master.clk_speed = 400000; // 100kHz
#endif
    while (1)
    {
        esp_err_t res;
        printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
        printf("00:         ");
        for (uint8_t addr = 3; addr < 0x78; addr++)
        {
            if (addr % 16 == 0)
                printf("\n%.2x:", addr);

            dev.addr = addr;
            res = i2c_dev_probe(&dev, I2C_DEV_WRITE);

            if (res == 0)
                printf(" %.2x", addr);
            else
                printf(" --");
        }
        printf("\n\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    
    mesure_encours = false ; 
    mesure_encours2 = false ; 
    START_MESURE = false ;
    ESP_ERROR_CHECK(i2cdev_init());
    config_mpu(&dev1,&dev2) ;

    ESP_ERROR_CHECK(servo_init(&esc,GPIO_NUM_23,LEDC_CHANNEL_0)) ;
    //ESP_LOGI(TAG, "Création de l'évenement") ;
    //data_rdy = xEventGroupCreate();
    acq_evt = xEventGroupCreate();
    ESP_ERROR_CHECK(gpio_install_isr_service(0)) ;
    ESP_LOGI(TAG, "Création de la tache") ;
    
  	xTaskCreatePinnedToCore(task_lecture_accel1, "Lecture accelero 1", configMINIMAL_STACK_SIZE * 8, NULL, (configMAX_PRIORITIES -1), &read_accel,1);
    xTaskCreatePinnedToCore(task_lecture_accel2, "Lecture accelero 2", configMINIMAL_STACK_SIZE * 8, NULL, (configMAX_PRIORITIES -1), &read_accel2,0);

    xTaskCreatePinnedToCore(task_regul_rpm, "Regulation RPM", configMINIMAL_STACK_SIZE * 8, NULL, (configMAX_PRIORITIES -5), &regul_rpm,0);
    
    
    /** test i2c  */
    //xTaskCreate(task, "i2c_scanner", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    
    //start_time = esp_timer_get_time() ;
    /* Configure the peripheral according to the LED type */


    gpio_config_t io_conf_laser_isr = {
        .pin_bit_mask = (1ULL << LASER_INT_PIN), // Sélection de la broche
        .mode = GPIO_MODE_INPUT,                   // Mode Entrée
        .pull_up_en = GPIO_PULLUP_ENABLE,          // Active la résistance de Pull-up interne
        .pull_down_en = GPIO_PULLDOWN_DISABLE,      // Désactive la résistance de Pull-down
        .intr_type = GPIO_INTR_NEGEDGE             // Déclenchement sur Flanc Descendant (Falling Edge)
    };

    vTaskDelay(100 / portTICK_PERIOD_MS);


    ESP_LOGI(TAG, "Config GPIO 4") ;
    ESP_ERROR_CHECK(gpio_config(&io_conf_laser_isr));
    // 3. Installer le service d'interruption global du périphérique GPIO
    
    //gpio_intr_enable(LASER_INT_PIN) ;
    // 4. Associer notre fonction ISR à la broche spécifique



    ESP_LOGI(TAG, "Config handler laser") ;
    ESP_ERROR_CHECK(gpio_isr_handler_add(LASER_INT_PIN, laser_isr, (void*) LASER_INT_PIN)) ;
    ESP_ERROR_CHECK(gpio_intr_enable(LASER_INT_PIN)) ;
    
    vTaskDelay(500 / portTICK_PERIOD_MS); 
    xTaskCreate(uart_console_task, "uart_console_task", 4096, NULL, 5, NULL);

    while (1) {
        //printf("Consigne RPM : %ld - Laser RPM : %ld - PPM : %ld\n",RPM,laser_rpm,ppm) ;
        vTaskDelay(500 / portTICK_PERIOD_MS); 
    }
}