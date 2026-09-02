/* Tesseract IMU firewall. Host 1 kHz. Resonator ω_n is 100 Hz.
   Not machine.c. Do not steal body 12.
   Bias is in the Euler. drive_velocity_m_s is sense-axis speed for F_c.
   make && ./tesseract */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "../../include/ztp.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static void init_orb(ZtpTesseractState *s, double alpha)
{
    memset(s, 0, sizeof(*s));
    s->mass_kg = 1.0e-7;
    s->omega_n_rad_s = 2.0 * M_PI * 100.0;
    s->zeta = 0.05;
    s->alpha = alpha;
}

int main(void)
{
    _Static_assert(sizeof(ZtpTesseractState) == 64, "ZtpTesseractState is 8 x f64");
    _Static_assert(sizeof(ZtpTesseractTick) == 48, "ZtpTesseractTick is 5 x f64 + 3 bool");
    const double dt = 0.001; /* chassis firewall — not a 1000 Hz constitutive look */
    const double w = 2.0 * M_PI * 100.0;
    const double w2 = w * w;

    printf("ztp-runtime eval  ·  tesseract IMU  ·  firewall 1 kHz  ·  ω_n = 100 Hz\n");
    printf("  sizeof(ZtpTesseractState)=%zu  not machine.c  body 12 reserved\n",
           sizeof(ZtpTesseractState));

    /* Plateau: α=0, v~1 m/s, bias in budget. Must accept. */
    ZtpTesseractState plateau;
    init_orb(&plateau, 0.0);
    ZtpTesseractTick tk;
    memset(&tk, 0, sizeof(tk));
    int32_t code = ZTP_OK;
    int32_t plateau_code = ZTP_OK;
    for (int k = 0; k < 100; k++) {
        double t = (double)k * dt;
        double a = 40.0 * sin(w * t);
        code = ztp_tesseract_step(
            &plateau, a, 0.4, 0.0, 0.0, 1.2e6, 1.6e3, 0.05, 1.0, dt, &tk);
        if (code != ZTP_OK)
            break;
    }
    plateau_code = code;
    printf("  plateau  code=%d  x=%.6f  Fc=%.3e  hold=%d  nl=%d  floor=%d\n",
           (int)plateau_code, tk.displacement_m, tk.scale_factor_n,
           (int)tk.hold_ok, (int)tk.nonlinear, (int)tk.bias_floor_broken);

    /* Gate: tether + α AND ½|b|t² ≥ 0.05 m. Reject. Freeze. */
    ZtpTesseractState cliff;
    init_orb(&cliff, 1.0e9);
    cliff.displacement_m = 0.004;
    cliff.velocity_m_s = 0.8;
    cliff.time_s = 0.40;
    cliff.control_u = 12.0;
    double x0 = cliff.displacement_m;
    double v0 = cliff.velocity_m_s;
    double t0 = cliff.time_s;
    double u0 = cliff.control_u;
    code = ztp_tesseract_step(
        &cliff, 40.0, 0.5, 0.01, 0.0, 1.2e6, 1.6e3, 1.0, 70.0, dt, &tk);
    int frozen = (cliff.displacement_m == x0)
        && (cliff.velocity_m_s == v0)
        && (cliff.time_s == t0)
        && (cliff.control_u == u0);
    printf("  anomaly  code=%d  PHYSICAL_ANOMALY=%d  frozen=%d  nl=%d  floor=%d  ½bt²=%.4f\n",
           (int)code, ZTP_PHYSICAL_ANOMALY, frozen,
           (int)tk.nonlinear, (int)tk.bias_floor_broken, tk.bias_floor_m);

    /* Independent columns: tether alone is OK; plateau + broken floor is OK. */
    ZtpTesseractState tether;
    init_orb(&tether, 1.0e9);
    tether.time_s = 0.40;
    int32_t c_tether = ztp_tesseract_step(
        &tether, 10.0, 0.4, 0.0, 0.0, 1.2e6, 1.6e3, 0.05, 70.0, dt, &tk);
    printf("  tether-only  code=%d  nl=%d  floor=%d  (must be OK)\n",
           (int)c_tether, (int)tk.nonlinear, (int)tk.bias_floor_broken);

    ZtpTesseractState bias;
    init_orb(&bias, 0.0);
    bias.time_s = 0.40;
    int32_t c_bias = ztp_tesseract_step(
        &bias, 10.0, 0.4, 0.0, 0.0, 1.2e6, 1.6e3, 1.0, 1.0, dt, &tk);
    printf("  floor-only   code=%d  nl=%d  floor=%d  (must be OK)\n",
           (int)c_bias, (int)tk.nonlinear, (int)tk.bias_floor_broken);

    printf("  hold  kp=1.2e6 %s ω_n²=%.3e\n",
           (1.2e6 > w2) ? ">" : "<=", w2);

    /* Sense-axis 70×: same Ω, tether vs linear drive_v. Not well ẋ. */
    ZtpTesseractState st_lin, st_teth;
    ZtpTesseractTick tk_lin, tk_teth;
    init_orb(&st_lin, 0.0);
    init_orb(&st_teth, 0.0);
    memset(&tk_lin, 0, sizeof(tk_lin));
    memset(&tk_teth, 0, sizeof(tk_teth));
    (void)ztp_tesseract_step(
        &st_lin, 0.0, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, dt, &tk_lin);
    (void)ztp_tesseract_step(
        &st_teth, 0.0, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, dt, &tk_teth);
    double fc_ratio = (tk_lin.scale_factor_n > 0.0)
        ? (tk_teth.scale_factor_n / tk_lin.scale_factor_n)
        : 0.0;
    printf("  scale  Fc_tether/Fc_lin=%.1f  (must be 70)\n", fc_ratio);

    int pass = (plateau_code == ZTP_OK)
        && (code == ZTP_PHYSICAL_ANOMALY) && frozen
        && (c_tether == ZTP_OK) && (c_bias == ZTP_OK)
        && (1.2e6 > w2)
        && (fc_ratio > 69.0) && (fc_ratio < 71.0);
    printf("  %s\n", pass ? "pass" : "FAIL");
    return pass ? 0 : 1;
}
