/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
//#include "sdkconfig.h"
#include <mpu6050.h>
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "math.h"
#include "esp_dsp.h"
#include "driver/uart.h"
#include "servo.h"

#define MPU6050_INT_PIN 5
#define LASER_INT_PIN 4
#define ADDR 0x68
#define SAMPLES 1024
#define RPM_ERROR 50
#define STABILIZATION_TIME 7000

struct data_mpu_t
{
float gyrox ;
float gyroy ;
float time ;
}data_mpu ;

typedef struct
{
    double f_ref;
    float amplitude;
    float phase ;
    int deg ;
    float angle ;
}data_output_t ;

typedef struct {
    float amp;        // amplitude du signal à f_ref (même unité que l'entrée)
    float phase_deg;  // phase par rapport à la référence (-180..180)
    float X, Y;       // composantes en phase / en quadrature
    int   n_used;     // nombre d'échantillons utilisés
} lockin_result_t;

servo_t esc ;
uint32_t RPM ;
uint32_t ppm ;
uint32_t laser_rpm ;
bool rpm_rdy ;
bool mesure_encours ;
uint16_t mesure ;

float data_x[2*SAMPLES] ;
float data_y[2*SAMPLES] ;

struct data_mpu_t raw_data[SAMPLES] ;
data_output_t data_output[5] ;


TaskHandle_t read_accel ;
TaskHandle_t regul_rpm ;
#define EVT_ACQ_DONE   (1 << 0)

static EventGroupHandle_t data_rdy;

bool is_first = 1 ;
uint64_t start_time ;
uint32_t freq_laser ;
uint64_t period ;
mpu6050_dev_t dev = { 0 };
uint32_t nb_sample = 0 ;
bool get_data = 0 ;


static const char *TAG = "example";

mpu6050_rotation_t gyro = { 0 };
mpu6050_acceleration_t accel = { 0 };
uint64_t current_time ;

static volatile uint64_t laser_first = 0, laser_last = 0;
static volatile uint32_t laser_count = 0;

static void IRAM_ATTR mpu6050_isr(void* arg)
{
    BaseType_t woken = pdFALSE;
    current_time = esp_timer_get_time() ;
    if(nb_sample == 0 ) {
        start_time = current_time ;
        laser_count = 0 ;       
    }
    xEventGroupSetBitsFromISR(data_rdy, EVT_ACQ_DONE, &woken);
    if (woken) portYIELD_FROM_ISR();
}



static void IRAM_ATTR laser_isr(void *arg)
{
    uint64_t now = esp_timer_get_time();
    
    if (laser_count == 0) 
        laser_first = now;
    period = now - laser_last ;
    freq_laser = 1000000U / period ;
    laser_rpm = freq_laser * 60 ;
    laser_last = now;

    laser_count++;
}

lockin_result_t lockin(const uint32_t *t_us, const float *x, int n, double f_ref, double t_ref_s)
{
    lockin_result_t r = {0};

    // Nombre entier de périodes -> évite les fuites spectrales
    double duration = (t_us[n - 1] - t_us[0]) * 1e-6;
    int periods = (int)floor(duration * f_ref);
    if (periods < 1) return r;                       // signal trop court
    double t_end = t_us[0] * 1e-6 + periods / f_ref;

    // Moyenne (retire l'offset DC) sur la même plage
    double mean = 0; int m = 0;
    for (int i = 0; i < n && t_us[i] * 1e-6 <= t_end; i++) { mean += x[i]; m++; }
    if (m < 2) return r;
    mean /= m;

    double sx = 0, sy = 0;
    for (int i = 0; i < m; i++) {
        double t   = t_us[i] * 1e-6 - t_ref_s;
        double arg = 2.0 * M_PI * f_ref * t;
        double v   = x[i] - mean;
        sx += v * cos(arg);
        sy += v * sin(arg);
    }
    sx *= 2.0 / m;      // le facteur 2 restitue l'amplitude crête
    sy *= 2.0 / m;

    r.X = sx;  r.Y = sy;
    r.amp = sqrtf(sx * sx + sy * sy);
    r.phase_deg = atan2(sy, sx) * 180.0 / M_PI;
    r.n_used = m;
    return r;
}

#define K_DEG_PER_HZ   2.59459f  

float obtenir_angle_corrige(float angle_brut_deg, float f_ref)
{
    // 1. Soustraction du retard linéaire
    float angle_corrige = angle_brut_deg - (K_DEG_PER_HZ * f_ref);
    
    // 2. Normalisation stricte entre 0° et 360°
    angle_corrige = fmodf(angle_corrige, 360.0f);
    if (angle_corrige < 0.0f) {
        angle_corrige += 360.0f;
    }
    
    return angle_corrige;
}


// Remplacer ces fonctions par votre logique réelle
void declencher_mesure(uint16_t c_rpm) {
    mesure_encours = true ;
    nb_sample = 0 ;
    laser_count = 0 ;
    ESP_ERROR_CHECK(gpio_intr_enable(LASER_INT_PIN)) ; 
    printf("\n[Console] Lancement de la mesure à %drpm...\n",c_rpm);
    RPM = c_rpm ;
    vTaskDelay(pdMS_TO_TICKS(1000));
    while(!rpm_rdy) vTaskDelay(pdMS_TO_TICKS(100)); ;
    printf("\nRPM stabilisés à %ldrpm...\n",laser_rpm);
    // Mettre votre flag de capture à true ou appeler votre routine de mesure ici
    

    ESP_ERROR_CHECK(gpio_intr_enable(MPU6050_INT_PIN)) ;
    vTaskResume(read_accel);
}

void envoyer_donnees_mesure(void) {
    printf("\n[Console] Envoi des donnees de la FFT...\n");
    // Mettre votre boucle de printf pour exporter le spectre de la FFT
    printf("###%d\n",mesure) ;
    printf("t;Y;t\n") ;
    for(int j=0;j<SAMPLES*2;j+=2)
    {
        //printf("%.0f;%f;%f\n",data_x[j],data_x[j+1],data_y[j+1]) ;
        printf("%.0f;%f\n",data_x[j],data_y[j+1]) ;
    }
    printf("###\n") ;
}

void lancer_les_mesure(void)
{
    uint16_t rpms[] = {1000,1500,1750,2000,2500} ;
    mesure = 0 ;
    printf("###sample=%d###\n",SAMPLES);
    while(mesure < 5)
    {
        if(!mesure_encours)
        {
            declencher_mesure(rpms[mesure]) ;
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    while(mesure_encours) vTaskDelay(pdMS_TO_TICKS(100));
    RPM = 0 ;
    printf("###lockin\n");
    printf("Freq;Amplitude;Phase;Deg;Angle\n") ;
    for(int i = 0;i<5;i++)
    {
        printf("%f;%f;%f;%d;%f\n",data_output[i].f_ref,data_output[i].amplitude,data_output[i].phase,data_output[i].deg,data_output[i].angle) ;
    }
    printf("###\n");
   

}

#define UART_CONSOLE_NUM UART_NUM_0
#define BUF_SIZE (1024)

static void uart_console_task(void *pvParameters)
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
            }
        }
        // Petit délai pour céder le processeur aux tâches prioritaires (ex: capture MPU)
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    free(data);
    vTaskDelete(NULL);
}

void config_mpu(void)
{
   
    ESP_LOGI(TAG, "Configuration du gyro") ;
    ESP_ERROR_CHECK(mpu6050_init_desc(&dev, ADDR, 0, 21, 22));

    while (1)
    {
        ESP_LOGI(TAG, "avant i2c_dev_probe") ;
        esp_err_t res = i2c_dev_probe(&dev.i2c_dev, I2C_DEV_WRITE);
        ESP_LOGI(TAG, "après i2c_dev_probe") ;
        if (res == ESP_OK)
        {
            ESP_LOGI(TAG, "Found MPU60x0 device");
            break;
        }
        ESP_LOGE(TAG, "MPU60x0 not found");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "Avant mpu6050_init") ;
    ESP_ERROR_CHECK(mpu6050_init(&dev));
    ESP_LOGI(TAG, "Après mpu6050_init") ;

    ESP_ERROR_CHECK(mpu6050_set_rate(&dev, 0)); // Set sample rate to 1 kHz.
    ESP_ERROR_CHECK(mpu6050_set_dlpf_mode(&dev, MPU6050_DLPF_3));
    ESP_ERROR_CHECK(mpu6050_set_full_scale_gyro_range(&dev, MPU6050_GYRO_RANGE_250));
    vTaskDelay(100 / portTICK_PERIOD_MS);
    //ESP_ERROR_CHECK(mpu6050_set_full_scale_accel_range(&dev, MPU6050_ACCEL_RANGE_2)) ;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Avant mpu6050_set_interrupt_mode") ;
    ESP_ERROR_CHECK(mpu6050_set_interrupt_mode(&dev,MPU6050_INT_LEVEL_HIGH)) ;
    ESP_ERROR_CHECK(mpu6050_set_interrupt_drive(&dev,MPU6050_INT_PUSH_PULL)) ;
    ESP_ERROR_CHECK(mpu6050_set_interrupt_latch(&dev,MPU6050_INT_LATCH_PULSE)) ;
    ESP_ERROR_CHECK(mpu6050_set_interrupt_latch_clear(&dev,1)) ;
    ESP_ERROR_CHECK(mpu6050_set_int_enabled(&dev,0x01)) ;
    ESP_LOGI(TAG, "Après mpu6050_get_interrupt_latch_clear") ;

    //ESP_LOGI(TAG, "Gyro range:  %d", dev.ranges.gyro);
    //ESP_LOGI(TAG, "Accel range:  %d", dev.ranges.accel);
    vTaskDelay(100 / portTICK_PERIOD_MS);

}

static float fft_buf[2 * SAMPLES];   // [re, im, re, im, ...]
static float window[SAMPLES];

void compute_fft(const struct data_mpu_t *raw, int n)
{
    // Fréquence d'échantillonnage réelle, déduite des timestamps (µs)
    float duration_us = raw[n - 1].time - raw[0].time;
    float fs = (n - 1) * 1e6f / duration_us;

    //printf("duration : %f - FS : %f\n",duration_us,fs) ;

    // Moyenne du signal, pour retirer l'offset DC du gyro
    float mean = 0;
    for (int i = 0; i < n; i++) mean += raw[i].gyroy;
    mean /= n;

    //printf("mean : %f\n",mean) ;
    // Fenêtre de Hann, réduit les fuites spectrales
    dsps_wind_hann_f32(window, n);
    /*for (int i = 0; i < n; i++) {
        printf("%f\n",window[i]) ;
    }    */

    for (int i = 0; i < n; i++) {
        fft_buf[2 * i]     = (raw[i].gyroy - mean) * window[i];  // partie réelle
        fft_buf[2 * i + 1] = 0.0f;                               // partie imaginaire
    }

    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, n));  // une seule fois suffit
    dsps_fft2r_fc32(fft_buf, n);
    dsps_bit_rev_fc32(fft_buf, n);

    // Seule la moitié du spectre est utile (0 à fs/2)
    float max_mag = 0;
    int max_k = 0;
    for (int k = 1; k < n / 2; k++) {
        float re = fft_buf[2 * k];
        float im = fft_buf[2 * k + 1];
        float mag = sqrtf(re * re + im * im) / n;
        //printf("%.2f Hz : %.4f\n", k * fs / n, mag);
        if (mag > max_mag) { max_mag = mag; max_k = k; }
    }
    printf("fs = %.1f Hz, pic à %.2f Hz\n", fs, max_k * fs / n);
}

void task_lecture_donnee()
{
    while(1) {
        while(nb_sample < SAMPLES){
            EventBits_t bits = xEventGroupWaitBits(
            data_rdy,
            EVT_ACQ_DONE,      // bits attendus
            pdTRUE,            // effacer les bits après réveil
            pdFALSE,           // pdTRUE = attendre TOUS les bits, pdFALSE = au moins un
            portMAX_DELAY);    // timeout

            ESP_ERROR_CHECK(mpu6050_get_acceleration(&dev, &accel));
            raw_data[nb_sample].gyrox = accel.x ;
            raw_data[nb_sample].gyroy = accel.y ;
            
            //ESP_LOGI(TAG, "Get Gyro") ;
            /*ESP_ERROR_CHECKmpu6050_get_rotation(&dev, &gyro));
            raw_data[nb_sample].gyrox = gyro.z ;
            raw_data[nb_sample].gyroy = gyro.y ; */


            raw_data[nb_sample].time = (float)current_time - (float)start_time ; 
            get_data = 0 ;
            nb_sample++ ;
            if (bits & EVT_ACQ_DONE) {
                // acquisition terminée
            }
        }
        gpio_intr_disable(MPU6050_INT_PIN) ;
        //gpio_intr_disable(LASER_INT_PIN) ;

        printf("Fin de l'acquisition") ;

        //Préparation des tableau pour la FFT
        printf("Préparation de la FFT") ;
        for(int i=0;i<SAMPLES;i++)
        {
            data_x[i*2]     = raw_data[i].time ;
            data_y[i*2]     = raw_data[i].time ;
            data_x[1+2*i]   = raw_data[i].gyrox ;
            data_y[1+2*i]   = raw_data[i].gyroy ;
        }

        //printf("t;X;Y;t\n") ;
        for(int j=0;j<SAMPLES*2;j+=2)
        {
            //printf("%.0f;%f;%f\n",data_x[j],data_x[j+1],data_y[j+1]) ;
            //printf("%.0f;%f\n",data_x[j],data_y[j+1]) ;
        }
     
        compute_fft(raw_data, SAMPLES);

        
        double f_ref = (laser_count - 1) * 1e6 / (double)(laser_last - laser_first);
        double t_laser0 = ((double)((int64_t)laser_first - (int64_t)start_time)) * 1e-6; // en s, base de temps des échantillons

        static float sig[SAMPLES];
        static uint32_t tus[SAMPLES];
        for (int i = 0; i < SAMPLES; i++) {
            tus[i] = raw_data[i].time;
            sig[i] = raw_data[i].gyroy;      // ou gyrox, etc.
        }

        lockin_result_t r = lockin(tus, sig, SAMPLES, f_ref, t_laser0); 
        float angle_corrige = obtenir_angle_corrige(r.phase_deg,f_ref) ;

        data_output[mesure].f_ref = f_ref ;
        data_output[mesure].amplitude = r.amp ;
        data_output[mesure].phase = r.phase_deg ;
        data_output[mesure].deg = r.n_used ; 
        data_output[mesure].angle = angle_corrige ;
        mesure++;

        printf("Laser count : %ld | f_ref = %.2f Hz | amplitude = %.4f | phase = %.1f deg (n=%d) | angle : %.1f\n",laser_count,f_ref, r.amp, r.phase_deg, r.n_used,angle_corrige);
        mesure_encours = false ;
        envoyer_donnees_mesure() ;
        vTaskSuspend(NULL) ;
    }

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
            ESP_ERROR_CHECK(servo_set_pulse_us(&esc,ppm)) ;
            //vTaskDelay(50 / portTICK_PERIOD_MS); 
        }
        if(rpm_ok > 50)
            rpm_rdy = true ;
        else
            rpm_rdy = false ;
        vTaskDelay(80 / portTICK_PERIOD_MS); 
    }
    if((esp_timer_get_time() - laser_last) > 10000000)
        laser_rpm = 0 ;
}

void app_main(void)
{
 
    ESP_ERROR_CHECK(i2cdev_init());
    config_mpu() ;

    ESP_ERROR_CHECK(servo_init(&esc,GPIO_NUM_23,LEDC_CHANNEL_0)) ;
    ESP_LOGI(TAG, "Création de l'évenement") ;
    data_rdy = xEventGroupCreate();


    ESP_LOGI(TAG, "Création de la tache") ;
    xTaskCreate(uart_console_task, "uart_console_task", 4096, NULL, 5, NULL);
  	xTaskCreatePinnedToCore(task_lecture_donnee, "Lecture accelero", configMINIMAL_STACK_SIZE * 8, NULL, (configMAX_PRIORITIES -1), &read_accel,1);
    vTaskSuspend(read_accel);

    xTaskCreatePinnedToCore(task_regul_rpm, "Regulation RPM", configMINIMAL_STACK_SIZE * 8, NULL, (configMAX_PRIORITIES -5), &regul_rpm,0);
    
    start_time = esp_timer_get_time() ;
    /* Configure the peripheral according to the LED type */
    gpio_config_t io_conf_mpu6050_isr = {
        .pin_bit_mask = (1ULL << MPU6050_INT_PIN), // Sélection de la broche
        .mode = GPIO_MODE_INPUT,                   // Mode Entrée
        .pull_up_en = GPIO_PULLUP_ENABLE,          // Active la résistance de Pull-up interne
        .pull_down_en = GPIO_PULLDOWN_DISABLE,      // Désactive la résistance de Pull-down
        .intr_type = GPIO_INTR_POSEDGE             // Déclenchement sur Flanc Descendant (Falling Edge)
    };
    gpio_config_t io_conf_laser_isr = {
        .pin_bit_mask = (1ULL << LASER_INT_PIN), // Sélection de la broche
        .mode = GPIO_MODE_INPUT,                   // Mode Entrée
        .pull_up_en = GPIO_PULLUP_ENABLE,          // Active la résistance de Pull-up interne
        .pull_down_en = GPIO_PULLDOWN_DISABLE,      // Désactive la résistance de Pull-down
        .intr_type = GPIO_INTR_NEGEDGE             // Déclenchement sur Flanc Descendant (Falling Edge)
    };

    vTaskDelay(100 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Config GPIO 5") ;
    ESP_ERROR_CHECK(gpio_config(&io_conf_mpu6050_isr));
    ESP_LOGI(TAG, "Config GPIO 4") ;
    ESP_ERROR_CHECK(gpio_config(&io_conf_laser_isr));
    // 3. Installer le service d'interruption global du périphérique GPIO
    
    //gpio_intr_enable(LASER_INT_PIN) ;
    // 4. Associer notre fonction ISR à la broche spécifique
    ESP_ERROR_CHECK(gpio_install_isr_service(0)) ;
    ESP_LOGI(TAG, "Config handler mpu") ;
    ESP_ERROR_CHECK(gpio_isr_handler_add(MPU6050_INT_PIN, mpu6050_isr, (void*) MPU6050_INT_PIN));
    ESP_ERROR_CHECK(gpio_intr_enable(MPU6050_INT_PIN)) ;

    ESP_LOGI(TAG, "Config handler laser") ;
    gpio_isr_handler_add(LASER_INT_PIN, laser_isr, (void*) LASER_INT_PIN);
    ESP_ERROR_CHECK(gpio_intr_enable(LASER_INT_PIN)) ;
    
    //declencher_mesure() ;
    while (1) {
        //printf("Consigne RPM : %ld - Laser RPM : %ld - PPM : %ld\n",RPM,laser_rpm,ppm) ;
        vTaskDelay(500 / portTICK_PERIOD_MS); 
    }
}