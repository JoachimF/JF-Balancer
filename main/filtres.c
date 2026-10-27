#include <stdio.h>
#include "math.h"
#include "esp_dsp.h"
#include "filtres.h"

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

float obtenir_amplitude_corrige1(float amplitude_brute, float f_ref)
{
    // Évite la division par zéro ou les aberrations à l'arrêt
    if (f_ref < 2.0f) {
        return 0.0f;
    }
    
    // Calcul du ratio de vitesse (w / w_ref)
    float ratio = f_ref / 33;
    
    // Correction quadratique : l'accélération mécanique augmente avec le carré de la vitesse
    float amp_corrigee = amplitude_brute / (ratio);
    
    return amp_corrigee;
}

float obtenir_amplitude_corrige2(float amplitude_brute, float f_ref)
{
    // Évite la division par zéro ou les aberrations à l'arrêt
    if (f_ref < 2.0f) {
        return 0.0f;
    }
    
    // Calcul du ratio de vitesse (w / w_ref)
    float ratio = f_ref / 33;
    
    // Correction quadratique : l'accélération mécanique augmente avec le carré de la vitesse
    float amp_corrigee = amplitude_brute / (ratio);
    
    return amp_corrigee;
}

float obtenir_angle_corrige1(float angle_brut_deg, float f_ref)
{
    // 1. Soustraction du retard linéaire
    float angle_corrige = angle_brut_deg - (K_DEG_PER_HZ_1 * f_ref);
    
    // 2. Normalisation stricte entre 0° et 360°
    angle_corrige = fmodf(angle_corrige, 360.0f);
    if (angle_corrige < 0.0f) {
        angle_corrige += 360.0f;
    }
    
    return angle_corrige;
}

float obtenir_angle_corrige2(float angle_brut_deg, float f_ref)
{
    // 1. Soustraction du retard linéaire
    float angle_corrige = angle_brut_deg - (K_DEG_PER_HZ_2 * f_ref);
    
    // 2. Normalisation stricte entre 0° et 360°
    angle_corrige = fmodf(angle_corrige, 360.0f);
    if (angle_corrige < 0.0f) {
        angle_corrige += 360.0f;
    }
    
    return angle_corrige;
}

static float fft_buf[2 * SAMPLES];   // [re, im, re, im, ...]
static float window[SAMPLES];

void compute_fft(const data_mpu_t *raw, int n)
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