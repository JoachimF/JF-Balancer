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
#include "sdkconfig.h"
#include <mpu6050.h>
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "math.h"
#include "esp_dsp.h"

#define MPU6050_INT_PIN 5
#define LASER_INT_PIN 4
#define ADDR 0x68
#define SAMPLES 1024

struct data_mpu_t
{
float gyrox ;
float gyroy ;
float time ;
}data_mpu ;

typedef struct {
    float amp;        // amplitude du signal à f_ref (même unité que l'entrée)
    float phase_deg;  // phase par rapport à la référence (-180..180)
    float X, Y;       // composantes en phase / en quadrature
    int   n_used;     // nombre d'échantillons utilisés
} lockin_result_t;

float data_x[2*SAMPLES] ;
float data_y[2*SAMPLES] ;

struct data_mpu_t raw_data[SAMPLES] ;

bool is_first = 1 ;
uint64_t start_time ;
mpu6050_dev_t dev = { 0 };
uint32_t nb_sample = 0 ;
bool get_data = 0 ;


static const char *TAG = "example";

mpu6050_rotation_t gyro = { 0 };
mpu6050_acceleration_t accel = { 0 };
uint64_t current_time ;

static void IRAM_ATTR mpu6050_isr(void* arg)
{
    //uint32_t gpio_num = (uint32_t) arg;
    current_time = esp_timer_get_time() ;
    get_data = 1 ;
}

static volatile uint64_t laser_first = 0, laser_last = 0;
static volatile uint32_t laser_count = 0;

static void IRAM_ATTR laser_isr(void *arg)
{
    uint64_t now = esp_timer_get_time();
    if (laser_count == 0) laser_first = now;
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

    ESP_LOGI(TAG, "avant mpu6050_init") ;
    ESP_ERROR_CHECK(mpu6050_init(&dev));
    ESP_LOGI(TAG, "avant mpu6050_init") ;

    ESP_ERROR_CHECK(mpu6050_set_rate(&dev, 0)); // Set sample rate to 1 kHz.
    ESP_ERROR_CHECK(mpu6050_set_dlpf_mode(&dev, MPU6050_DLPF_3));
    ESP_ERROR_CHECK(mpu6050_set_full_scale_gyro_range(&dev, MPU6050_GYRO_RANGE_250));

    ESP_LOGI(TAG, "avant mpu6050_set_interrupt_mode") ;
    ESP_ERROR_CHECK(mpu6050_set_interrupt_mode(&dev,MPU6050_INT_LEVEL_HIGH)) ;
    ESP_ERROR_CHECK(mpu6050_set_interrupt_drive(&dev,MPU6050_INT_PUSH_PULL)) ;
    ESP_ERROR_CHECK(mpu6050_set_interrupt_latch(&dev,MPU6050_INT_LATCH_PULSE)) ;
    ESP_ERROR_CHECK(mpu6050_set_interrupt_latch_clear(&dev,1)) ;
    ESP_ERROR_CHECK(mpu6050_set_int_enabled(&dev,0x01)) ;
    ESP_LOGI(TAG, "après mpu6050_get_interrupt_latch_clear") ;

    ESP_LOGI(TAG, "Gyro range:  %d", dev.ranges.gyro);

}

static float fft_buf[2 * SAMPLES];   // [re, im, re, im, ...]
static float window[SAMPLES];

void compute_fft(const struct data_mpu_t *raw, int n)
{
    // Fréquence d'échantillonnage réelle, déduite des timestamps (µs)
    float duration_us = raw[n - 1].time - raw[0].time;
    float fs = (n - 1) * 1e6f / duration_us;

    // Moyenne du signal, pour retirer l'offset DC du gyro
    float mean = 0;
    for (int i = 0; i < n; i++) mean += raw[i].gyrox;
    mean /= n;

    // Fenêtre de Hann, réduit les fuites spectrales
    dsps_wind_hann_f32(window, n);

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

void app_main(void)
{

    
    ESP_ERROR_CHECK(i2cdev_init());
    config_mpu() ;
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
    start_time = esp_timer_get_time() ;
    
    nb_sample = 0 ;
    laser_count = 0 ;
    while (1) {
        /*while (1) 
        {
            printf("X : %ld\n",laser_count) ;
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }*/

        ESP_LOGI(TAG, "Attente des données") ;
        while(nb_sample < SAMPLES){
            if(get_data)
            {
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
                //printf("X : %f - Y : %f - t : %f\n",gyro.x,gyro.y,raw_data[nb_sample].time) ;
                //ESP_LOGI(TAG, "Sample reçu : %ld",nb_sample) ;
            }
            //vTaskDelay(10 / portTICK_PERIOD_MS);
            
        }
            
        
        gpio_intr_disable(MPU6050_INT_PIN) ;
        gpio_intr_disable(LASER_INT_PIN) ;

        double f_ref = (laser_count - 1) * 1e6 / (double)(laser_last - laser_first);
        double t_laser0 = ((double)((int64_t)laser_first - (int64_t)start_time)) * 1e-6; // en s, base de temps des échantillons

        //Préparation des tableau pour la FFT
        ESP_LOGI(TAG, "Préparation de la FFT") ;
        for(int i=0;i<SAMPLES;i++)
        {
            data_x[i*2]     = raw_data[i].time ;
            data_y[i*2]     = raw_data[i].time ;
            data_x[1+2*i]   = raw_data[i].gyrox ;
            data_y[1+2*i]   = raw_data[i].gyroy ;
        }

        printf("Laser count : %ld\n",laser_count) ;
        //printf("t;X;Y;t\n") ;
        for(int j=0;j<SAMPLES*2;j+=2)
        {
            //printf("%f;%f;%f\n",data_x[j],data_x[j+1],data_y[j+1]) ;
        }

        static float sig[SAMPLES];
        static uint32_t tus[SAMPLES];
        for (int i = 0; i < SAMPLES; i++) {
            tus[i] = raw_data[i].time;
            sig[i] = raw_data[i].gyroy;      // ou gyrox, etc.
        }

        printf("Laser count : %ld\n",laser_count) ;
        lockin_result_t r = lockin(tus, sig, SAMPLES, f_ref, t_laser0);     
        printf("f_ref = %.2f Hz | amplitude = %.4f | phase = %.1f deg (n=%d)\n",f_ref, r.amp, r.phase_deg, r.n_used);


        compute_fft(raw_data, SAMPLES);

        while (1) vTaskDelay(10 / portTICK_PERIOD_MS); 
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
