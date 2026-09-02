/* 1.2 N liver policy. Do not destroy the sample. make && ./tissue */
#include <stdio.h>
#include <stdbool.h>
#include "../../include/ztp.h"

int main(void) {
    ZtpSurgicalTissueAuditor a = {
        .tissue_type_id = 0,          /* liver / spleen */
        .max_tearing_force_n = 1.2f,
        .measured_displacement_m = 0.0f,
        .measured_force_n = 0.40f,
        .relaxation_tau = 0.15f,
        .last_displacement_m = 0.0f,
        .last_force_n = 0.40f,
        .accumulated_energy_j = 0.0f,
    };
    const float dt = 0.001f;
    int first_overstress = -1, first_arrest = -1, first_rupture = -1, first_cable = -1;
    int reflex = 0;
    float clamp_n = 1.2f;

    printf("ztp-runtime eval  ·  tissue  ·  liver 1.2 N  ·  1000 Hz\n");
    printf("  policy ramps 0.40 → 2.20 N. overstress freezes the jaw and decays to clamp.\n");
    for (int step = 0; step < 100; step++) {
        a.last_displacement_m = a.measured_displacement_m;
        a.last_force_n = a.measured_force_n;
        if (!reflex) {
            a.measured_force_n = 0.40f + 0.018f * (float)step;
            a.measured_displacement_m = 0.00004f * (float)step;
        } else {
            /* dx = 0 so rupture detector does not fire on the back-off */
            a.measured_force_n += 0.12f * (clamp_n - a.measured_force_n);
        }

        ZtpSurgicalResult r = ztp_surgical_evaluate_grasp(&a, dt);
        if (first_overstress < 0 && r.tissue_overstress_detected) {
            first_overstress = step;
            reflex = 1;
            clamp_n = r.clamped_force * 0.95f;
        }
        if (reflex && first_arrest < 0 && !r.tissue_overstress_detected)
            first_arrest = step;
        if (first_rupture < 0 && r.viscoelastic_rupture_detected) first_rupture = step;
        if (first_cable < 0 && r.cable_slip_fault) first_cable = step;

        if (step == 0 || step == first_overstress || step == first_arrest || step == 99) {
            printf("  t=%3d ms  F=%5.2f N  clamp=%.2f N  overstress=%d rupture=%d cable=%d reflex=%d\n",
                   step, a.measured_force_n, r.clamped_force,
                   r.tissue_overstress_detected, r.viscoelastic_rupture_detected,
                   r.cable_slip_fault, reflex);
        }
    }
    int halt_ms = (first_overstress >= 0 && first_arrest >= 0)
        ? first_arrest - first_overstress : -1;
    printf("  first overstress @ %d ms  arrest @ %d  halt=%d ms  rupture @ %d  cable @ %d\n",
           first_overstress, first_arrest, halt_ms, first_rupture, first_cable);
    printf("  %s\n", (halt_ms >= 0 && first_rupture < 0)
           ? "ethic — jaw froze, force back under 1.2 N, sample not a payload"
           : "check: liver halt did not bind");
    return 0;
}
