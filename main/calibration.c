#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <complex.h>
#include "math.h"
#include "calibration.h"

extern uint16_t mesure ;
extern uint32_t RPM ;
extern int declencher_mesure(uint16_t c_rpm) ;

cplx last_V1, last_V2;            // dernière mesure lockin (X + jY)
static cplx cal_V0[2], cal_VA[2], cal_VB[2];
static bool have_0 = false, have_A = false, have_B = false;
static cplx cal_corrA, cal_corrB;
static bool have_corr = false;

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
    RPM = 0;                              // moteur à l'arrêt pour pouvoir poser les masses
    if (ok < CAL_NB_MOYENNES) return false;
    *V1 = s1 / ok;
    *V2 = s2 / ok;
    return true;
}

static void print_v(const char *nom, cplx V)
{
    double ang = carg(V) * 180.0 / M_PI;
    if (ang < 0) ang += 360.0;
    printf("%s : amp=%.4f phase=%.1f deg\n", nom, cabs(V), ang);
}

static cplx mass_c(double g, double deg) { return g * cexp(I * deg * M_PI / 180.0); }

void cal_run0(void)
{
    printf("\n[CAL] Run 0 : rotor SANS masse d'essai\n");
    if (!mesure_moyenne(CAL_RPM, &cal_V0[0], &cal_V0[1])) { printf("[CAL] echec\n"); return; }
    have_0 = true; have_A = have_B = have_corr = false;
    print_v("V1_0", cal_V0[0]); print_v("V2_0", cal_V0[1]);
    printf("[CAL] Pose %.1f g a %.0f deg dans le plan A, puis tape 'a'\n", MASSE_ESSAI_A_G, ANGLE_ESSAI_A);
}

void cal_runA(void)
{
    if (!have_0) { printf("[CAL] fais d'abord le run 0 ('0')\n"); return; }
    printf("\n[CAL] Run A : masse d'essai dans le plan A\n");
    if (!mesure_moyenne(CAL_RPM, &cal_VA[0], &cal_VA[1])) { printf("[CAL] echec\n"); return; }
    have_A = true;
    print_v("V1_A", cal_VA[0]); print_v("V2_A", cal_VA[1]);
    printf("[CAL] RETIRE la masse A. Pose %.1f g a %.0f deg dans le plan B, puis tape 'b'\n",
           MASSE_ESSAI_B_G, ANGLE_ESSAI_B);
}

void cal_runB(void)
{
    if (!have_A) { printf("[CAL] fais d'abord le run A ('a')\n"); return; }
    printf("\n[CAL] Run B : masse d'essai dans le plan B\n");
    if (!mesure_moyenne(CAL_RPM, &cal_VB[0], &cal_VB[1])) { printf("[CAL] echec\n"); return; }
    have_B = true;
    print_v("V1_B", cal_VB[0]); print_v("V2_B", cal_VB[1]);
    printf("[CAL] RETIRE la masse B, puis tape 'k' pour calculer les corrections\n");
}

void cal_calcul(void)
{
    if (!(have_0 && have_A && have_B)) { printf("[CAL] calibration incomplete\n"); return; }

    cplx mA = mass_c(MASSE_ESSAI_A_G, ANGLE_ESSAI_A);
    cplx mB = mass_c(MASSE_ESSAI_B_G, ANGLE_ESSAI_B);
    cplx a11 = (cal_VA[0] - cal_V0[0]) / mA;
    cplx a21 = (cal_VA[1] - cal_V0[1]) / mA;
    cplx a12 = (cal_VB[0] - cal_V0[0]) / mB;
    cplx a22 = (cal_VB[1] - cal_V0[1]) / mB;

    cplx det = a11 * a22 - a12 * a21;
    double cond = cabs(det) / (cabs(a11) * cabs(a22) + 1e-12);
    printf("[CAL] conditionnement |det|/(|a11||a22|) = %.3f %s\n", cond,
           cond < 0.2 ? "(MAUVAIS : plans mal separes)" : "(ok)");
    if (cabs(det) < 1e-12) { printf("[CAL] matrice singuliere\n"); return; }

    cal_corrA = -( a22 * cal_V0[0] - a12 * cal_V0[1]) / det;
    cal_corrB = -(-a21 * cal_V0[0] + a11 * cal_V0[1]) / det;
    have_corr = true;

    double angA = carg(cal_corrA) * 180.0 / M_PI; if (angA < 0) angA += 360.0;
    double angB = carg(cal_corrB) * 180.0 / M_PI; if (angB < 0) angB += 360.0;
    printf("\n===== CORRECTION =====\n");
    printf("Plan A : %.2f g a %.1f deg\n", cabs(cal_corrA), angA);
    printf("Plan B : %.2f g a %.1f deg\n", cabs(cal_corrB), angB);
    printf("Pose ces masses, puis tape 'v' pour verifier.\n");
}

void cal_verif(void)
{
    if (!have_0) { printf("[CAL] pas de reference (run 0)\n"); return; }
    cplx V1, V2;
    printf("\n[CAL] Verification apres correction\n");
    if (!mesure_moyenne(CAL_RPM, &V1, &V2)) { printf("[CAL] echec\n"); return; }
    print_v("V1", V1); print_v("V2", V2);
    printf("Residu : capteur1 = %.0f %%, capteur2 = %.0f %% de l'initial\n",
           100.0 * cabs(V1) / cabs(cal_V0[0]), 100.0 * cabs(V2) / cabs(cal_V0[1]));
}