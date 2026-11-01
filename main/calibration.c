#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <complex.h>
#include <math.h>
#include "calibration.h"
#include "uart_console.h"

extern uint16_t mesure;
extern uint32_t RPM;
extern bool declencher_mesure(uint16_t c_rpm);

calibration_data_t calibration = {0};

cplx last_V1, last_V2;       // dernière mesure lockin (X + jY), écrite par calcul_des_resultats
static bool have_0 = false, have_A = false, have_B = false;

/* ------------------------------------------------------------------ outils */
static cplx polaire(double amp, double deg) { return amp * cexp(I * deg * M_PI / 180.0); }
static cplx mass_c(double g, double deg)    { return polaire(g, deg); }

static double angle360(cplx z)
{
    double a = carg(z) * 180.0 / M_PI;
    return a < 0 ? a + 360.0 : a;
}

static void print_v(const char *nom, cplx V)
{
    printf("%s : amp=%.5f phase=%.2f deg\n", nom, cabs(V), angle360(V));
}

// Mesure moyennée à vitesse fixe. Arrête le moteur à la fin.
static bool mesure_moyenne(uint16_t rpm, cplx *V1, cplx *V2)
{
    cplx s1 = 0, s2 = 0;
    int ok = 0, essais = 0;
    while (ok < CAL_NB_MOYENNES && essais < CAL_NB_MOYENNES * 3) {
        essais++;
        mesure = 0;                       // évite de sortir de data_output[5]
        if (declencher_mesure(rpm)) {
            s1 += last_V1;
            s2 += last_V2;
            ok++;
        }
    }
    RPM = 0;                              // moteur à l'arrêt
    if (ok < CAL_NB_MOYENNES) return false;
    *V1 = s1 / ok;
    *V2 = s2 / ok;
    return true;
}

/* ------------------------------------------------------- matrice d'influence */
// Calcule H à partir de V0, VA, VB et des masses d'essai (en g.mm).
static int cal_matrice(const _param_t *param)
{
    cplx mA = mass_c(MASSE_ESSAI_A_G * param->rayon_test_A_mm, ANGLE_ESSAI_A);   // g.mm
    cplx mB = mass_c(MASSE_ESSAI_B_G * param->rayon_test_B_mm, ANGLE_ESSAI_B);   // g.mm

    calibration.a11 = (calibration.cal_VA[0] - calibration.cal_V0[0]) / mA;
    calibration.a21 = (calibration.cal_VA[1] - calibration.cal_V0[1]) / mA;
    calibration.a12 = (calibration.cal_VB[0] - calibration.cal_V0[0]) / mB;
    calibration.a22 = (calibration.cal_VB[1] - calibration.cal_V0[1]) / mB;

    calibration.det = calibration.a11 * calibration.a22 - calibration.a12 * calibration.a21;
    calibration.conditionnement = cabs(calibration.det) /
                                  (cabs(calibration.a11) * cabs(calibration.a22) + 1e-12);

    calibration.calibre = isfinite(creal(calibration.det)) && isfinite(cimag(calibration.det))
                          && cabs(calibration.det) >= 1e-12;

    if (!calibration.calibre) { printf("[CAL] matrice singuliere ou valeurs invalides\n"); return -1; }
    printf("[CAL] conditionnement |det|/(|a11||a22|) = %.4f %s\n", calibration.conditionnement,
           calibration.conditionnement < 0.2 ? "(MAUVAIS : plans mal separes)" : "(ok)");
    return 0;
}

/* --------------------------------------------------------- calcul du balourd */
static void remplir(balourd_t *b, cplx U, double rayon_mm)
{
    b->balourd   = cabs(U);                       // g.mm
    b->angle     = angle360(U);                   // côté lourd
    b->masse     = b->balourd / rayon_mm;         // g au rayon de correction
    b->angle_cor = fmod(b->angle + 180.0, 360.0); // à l'opposé
}

// V = H.U  ->  U = H^-1.V
int cal_calcul_balourd(cplx V1, cplx V2, const _param_t *param, balourd_t *balA, balourd_t *balB)
{
    if (!calibration.calibre) return -1;
    if (balA == NULL || balB == NULL) return -2;

    double rA = param->rayon_correction_A_mm;
    double rB = param->rayon_correction_B_mm;
    if (rA <= 0 || rB <= 0) return -3;

    cplx UA = ( calibration.a22 * V1 - calibration.a12 * V2) / calibration.det;
    cplx UB = (-calibration.a21 * V1 + calibration.a11 * V2) / calibration.det;

    remplir(balA, UA, rA);
    remplir(balB, UB, rB);
    return 0;
}

static void afficher_balourd(const char *plan, const balourd_t *b, double rayon_mm)
{
    printf("Plan %s : balourd %.4f g.mm a %.1f deg\n", plan, b->balourd, b->angle);
    printf("          -> AJOUTER %.4f g a %.1f deg (rayon %.0f mm)\n", b->masse, b->angle_cor, rayon_mm);
    printf("          ou RETIRER %.4f g a %.1f deg\n", b->masse, b->angle);
}

void cal_mesure_balourd(const _param_t *param)
{
    if (!calibration.calibre) {
        printf("[BAL] pas de calibration valide (lance 0, a, b puis k, ou cal_charger)\n");
        return;
    }
    cplx V1, V2;
    printf("\n[BAL] Mesure du balourd a %d tr/min\n", CAL_RPM);
    if (!mesure_moyenne(CAL_RPM, &V1, &V2)) { printf("[BAL] echec de la mesure\n"); return; }
    print_v("V1", V1);
    print_v("V2", V2);

    balourd_t bA, bB;
    int r = cal_calcul_balourd(V1, V2, param, &bA, &bB);
    if (r != 0) { printf("[BAL] erreur de calcul (%d)\n", r); return; }

    printf("\n===== BALOURD =====\n");
    afficher_balourd("A", &bA, param->rayon_correction_A_mm);
    afficher_balourd("B", &bB, param->rayon_correction_B_mm);
}

/* ------------------------------------------------------------- calibration */
void cal_run0(void)
{
    printf("\n[CAL] Run 0 : rotor SANS masse d'essai\n");
    if (!mesure_moyenne(CAL_RPM, &calibration.cal_V0[0], &calibration.cal_V0[1])) { printf("[CAL] echec\n"); return; }
    have_0 = true; have_A = have_B = false;
    calibration.calibre = false;                      // l'ancienne matrice n'est plus valide
    print_v("V1_0", calibration.cal_V0[0]); print_v("V2_0", calibration.cal_V0[1]);
    printf("[CAL] Pose %.3f g a %.0f deg dans le plan A, puis tape 'a'\n", MASSE_ESSAI_A_G, ANGLE_ESSAI_A);
}

void cal_runA(void)
{
    if (!have_0) { printf("[CAL] fais d'abord le run 0 ('0')\n"); return; }
    printf("\n[CAL] Run A : masse d'essai dans le plan A\n");
    if (!mesure_moyenne(CAL_RPM, &calibration.cal_VA[0], &calibration.cal_VA[1])) { printf("[CAL] echec\n"); return; }
    have_A = true;
    calibration.calibre = false;
    print_v("V1_A", calibration.cal_VA[0]); print_v("V2_A", calibration.cal_VA[1]);
    printf("[CAL] RETIRE la masse A. Pose %.3f g a %.0f deg dans le plan B, puis tape 'b'\n",
           MASSE_ESSAI_B_G, ANGLE_ESSAI_B);
}

void cal_runB(void)
{
    if (!have_A) { printf("[CAL] fais d'abord le run A ('a')\n"); return; }
    printf("\n[CAL] Run B : masse d'essai dans le plan B\n");
    if (!mesure_moyenne(CAL_RPM, &calibration.cal_VB[0], &calibration.cal_VB[1])) { printf("[CAL] echec\n"); return; }
    have_B = true;
    calibration.calibre = false;
    print_v("V1_B", calibration.cal_VB[0]); print_v("V2_B", calibration.cal_VB[1]);
    printf("[CAL] RETIRE la masse B, puis tape 'k' pour calibrer\n");
}

// 'k' : calcule la matrice, puis affiche la correction du rotor mesuré au run 0
void cal_calcul(const _param_t *param)
{
    if (!(have_0 && have_A && have_B)) { printf("[CAL] calibration incomplete\n"); return; }
    if (cal_matrice(param) != 0) return;

    balourd_t bA, bB;
    if (cal_calcul_balourd(calibration.cal_V0[0], calibration.cal_V0[1], param, &bA, &bB) != 0) return;
    printf("\n===== CORRECTION (rotor du run 0) =====\n");
    afficher_balourd("A", &bA, param->rayon_correction_A_mm);
    afficher_balourd("B", &bB, param->rayon_correction_B_mm);
    printf("Pose ces masses, puis lance la mesure du balourd pour verifier.\n");
}

// Recharge une calibration déjà faite (après un redémarrage, par exemple)
void cal_charger(const double ref[6][2], const _param_t *param)
{
    calibration.cal_V0[0] = polaire(ref[0][0], ref[0][1]);
    calibration.cal_V0[1] = polaire(ref[1][0], ref[1][1]);
    calibration.cal_VA[0] = polaire(ref[2][0], ref[2][1]);
    calibration.cal_VA[1] = polaire(ref[3][0], ref[3][1]);
    calibration.cal_VB[0] = polaire(ref[4][0], ref[4][1]);
    calibration.cal_VB[1] = polaire(ref[5][0], ref[5][1]);
    have_0 = have_A = have_B = true;
    cal_matrice(param);
}

void init_calibration(const _param_t *param)
{
    // Amplitude et phase (deg) lues dans les logs "V1_0 : amp=... phase=..." de chaque run.
    // Les phases à NAN sont à REMPLACER : tant qu'elles le sont, la calibration est refusée.
    static const double ref[6][2] = {
        {0.0932, 68.9},   // V1_0
        {0.0179, 301.5},   // V2_0
        {0.2174, 47.7},   // V1_A
        {0.0232, 301.9},   // V2_A
        {0.0980, 79.0},   // V1_B
        {0.1065, 32.0},   // V2_B
    };
    cal_charger(ref, param);
}

void cal_verif(void)
{
    if (!calibration.calibre) { printf("[CAL] pas de reference\n"); return; }
    cplx V1, V2;
    printf("\n[CAL] Verification apres correction\n");
    if (!mesure_moyenne(CAL_RPM, &V1, &V2)) { printf("[CAL] echec\n"); return; }
    print_v("V1", V1); print_v("V2", V2);
    printf("Residu : capteur1 = %.0f %%, capteur2 = %.0f %% de l'initial\n",
           100.0 * cabs(V1) / cabs(calibration.cal_V0[0]), 100.0 * cabs(V2) / cabs(calibration.cal_V0[1]));
}