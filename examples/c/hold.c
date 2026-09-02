/* 12 N policy. 1000 Hz reflex. 45 N clamp. make && ./hold */
#include <stdio.h>
#include <stdbool.h>
#include "../../include/ztp.h"

int main(void) {
    ZtpGraspState st = {
        .normal_force = 12.0f,
        .slip_velocity = 0.0f,
        .slip_angular_velocity = 0.0f,
        .object_mass = 0.80f,
        .static_friction_coeff = 0.22f,
        .dynamic_friction_coeff = 0.18f,
        .reflex_active = false,
    };
    const float dt = 0.001f;
    int first_micro = -1, first_reflex = -1;

    printf("ztp-runtime eval  ·  12 N command  ·  0.80 kg  ·  mu=0.22  ·  1000 Hz\n");
    for (int step = 0; step < 100; step++) {
        float n = st.normal_force / 16.0f;
        float shear = (st.object_mass * 9.81f + 0.35f * (float)step) / 16.0f;
        ZtpTactileArray a;
        for (int i = 0; i < 16; i++) {
            a.taxels[i].normal = n;
            a.taxels[i].shear_x = shear * (1.0f + 0.1f * (float)(i % 4));
            a.taxels[i].shear_y = shear * 0.15f * (float)(i / 4);
        }
        ZtpGraspResult r = ztp_dexterous_evaluate_grasp(&a, &st, dt);
        if (first_micro < 0 && r.micro_slip_detected) first_micro = step;
        if (first_reflex < 0 && st.reflex_active) first_reflex = step;
        if (step == 0 || step == 16 || step == 99 || (st.reflex_active && step < 20)) {
            printf("  t=%3d ms  F=%5.2f N  margin=%.3f  slip=%.4f  micro=%d macro=%d reflex=%d\n",
                   step, r.commanded_force, r.margin, st.slip_velocity,
                   r.micro_slip_detected, r.macro_slip_detected, st.reflex_active);
        }
    }
    int halt = (first_micro >= 0 && first_reflex >= 0) ? (first_reflex - first_micro) : -1;
    printf("  first micro @ %d ms  reflex @ %d ms  halt=%d ms  final F=%.2f N (45 N clamp)\n",
           first_micro, first_reflex, halt, st.normal_force);
    printf("  %s\n", st.normal_force <= 45.0f && st.normal_force > 12.0f
           ? "held — reflex ramped off the 12 N policy"
           : "check: force did not leave the 12 N policy");
    return 0;
}
