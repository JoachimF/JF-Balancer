#ifndef _FILTRES_H_
#define _FILTRES_H_

#define K_DEG_PER_HZ_1   0 //1.978955888f     //2.59459f  
//#define K_APM_PER_HZ_1   0.7372362f //0.007307951f

#define K_DEG_PER_HZ_2   0 //1.226708075f
     //2.59459f  
//#define K_APM_PER_HZ_2   0.7372362f //0.007307951f

#define SAMPLES 128

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

typedef struct {
float gyrox ;
float gyroy ;
float time ;
} data_mpu_t ;


lockin_result_t lockin(const uint32_t *t_us, const float *x, int n, double f_ref, double t_ref_s) ;
float obtenir_amplitude_corrige1(float amplitude_brute, float f_ref) ;
float obtenir_angle_corrige1(float angle_brut_deg, float f_ref) ;
float obtenir_amplitude_corrige2(float amplitude_brute, float f_ref) ;
float obtenir_angle_corrige2(float angle_brut_deg, float f_ref) ;
void compute_fft(const data_mpu_t *raw, int n) ;

#endif