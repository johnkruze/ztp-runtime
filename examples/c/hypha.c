/* Mycelial Kirchhoff. 10 Hz. SPECTRA MycelialState 8×f64 wired, not redesigned.
   make && ./hypha   — not on machine.c */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "../../include/ztp.h"

int main(void) {
    _Static_assert(sizeof(ZtpMycelialState) == 64, "ZtpMycelialState is 8 x f64");
    _Static_assert(sizeof(ZtpLastStateFrame) == 64, "mycelial file frame is 64 B");

    const float dt = 0.1f; /* 10 Hz Kirchhoff — not the grasp 1 kHz loop */
    const float health0 = 0.72f;
    const float density0 = 0.38f;
    const float tilling = 0.85f;
    int first_frag = -1;
    ZtpLastStateFrame frames[100];
    memset(frames, 0, sizeof(frames));

    printf("ztp-runtime eval  ·  mycelial Kirchhoff  ·  10 Hz  ·  dt=0.1  ·  Dark Window hyphae\n");
    printf("  sizeof(ZtpMycelialState)=%zu  (SPECTRA MycelialState 8 x f64 — wired, not redesigned)\n",
           sizeof(ZtpMycelialState));

    for (int step = 0; step < 100; step++) {
        float t = (float)step * dt;
        ZtpMycelialState s = ztp_mycelial_evaluate_state(health0, density0, tilling, t);
        bool frag = s.fragmented_flag > 0.5;
        bool below = s.percolation_index < 1.0 && !frag;
        if (first_frag < 0 && frag) first_frag = step;
        frames[step] = ztp_last_state_pack_mycelial(
            (uint32_t)(t * 1000.0f + 0.5f),
            (float)s.health_index,
            (float)s.hyphal_density,
            (float)s.percolation_index,
            (float)s.delivered_nutrient,
            (float)s.conductance_mean,
            (float)s.tilling_stress,
            frag,
            below
        );
        if (step == 0 || step == 20 || step == 99) {
            printf("  t=%5.1f s  health=%.3f  density=%.3f  perc=%.3f  delivered=%.3f  C=%.3f  frag=%d\n",
                   t, s.health_index, s.hyphal_density, s.percolation_index,
                   s.delivered_nutrient, s.conductance_mean, frag);
        }
    }

    ZtpMycelialState healthy = ztp_mycelial_evaluate_state(health0, density0, 0.0f, 70.0f);
    ZtpMycelialState tilled = ztp_mycelial_evaluate_state(health0, density0, tilling, 70.0f);
    ZtpMycelialState sick = ztp_mycelial_evaluate_state(0.28f, 0.75f, 0.0f, 70.0f);
    bool tfrag = tilled.fragmented_flag > 0.5;
    bool tbelow = tilled.percolation_index < 1.0 && !tfrag;
    frames[99] = ztp_last_state_pack_mycelial(
        70000,
        (float)tilled.health_index,
        (float)tilled.hyphal_density,
        (float)tilled.percolation_index,
        (float)tilled.delivered_nutrient,
        (float)tilled.conductance_mean,
        (float)tilled.tilling_stress,
        tfrag,
        tbelow
    );

    printf("  Kirchhoff minute (70 s, 700 x dt=0.1 inside evaluate)\n");
    printf("  sparse-healthy  health=%.3f  density=%.3f  delivered=%.3f  frag=%d\n",
           healthy.health_index, healthy.hyphal_density, healthy.delivered_nutrient,
           (int)(healthy.fragmented_flag > 0.5));
    printf("  dense-sick      health=%.3f  density=%.3f  delivered=%.3f  frag=%d  (health > density)\n",
           sick.health_index, sick.hyphal_density, sick.delivered_nutrient,
           (int)(sick.fragmented_flag > 0.5));
    printf("  tilled          health=%.3f  density=%.3f  delivered=%.3f  frag=%d  tilling=%.2f\n",
           tilled.health_index, tilled.hyphal_density, tilled.delivered_nutrient,
           tfrag, tilled.tilling_stress);
    printf("  first fragmented @ %s  (tilling severs after 20 s of the minute)\n",
           first_frag >= 0 ? "this 10 s window" : "T_ESTABLISH=20 s (not in 100-step window)");

    const char *soma_path = "../../soma/mycelial_terminal.soma.bin";
    int wrote = ztp_last_state_write_mycelial(soma_path, frames, 100);
    printf("  wrote %s  ok=%d  body 8 MYCELIA1  100 frames\n", soma_path, wrote);

    FILE *f = fopen(soma_path, "rb");
    if (f) {
        uint8_t buf[64 + 100 * 64];
        size_t nread = fread(buf, 1, sizeof(buf), f);
        fclose(f);
        ZtpLastStateFrame last;
        memset(&last, 0, sizeof(last));
        int peek_ok = (nread == sizeof(buf))
            && ztp_last_state_header_ok(buf, (uint64_t)nread)
            && ztp_last_state_peek_last(buf, (uint64_t)nread, &last);
        uint16_t bid = (nread >= 64) ? ztp_last_state_body_id(buf) : 0;
        printf("  peek_ok=%d  body_id=%u  health=%.3f  density=%.3f  perc=%.3f\n",
               peek_ok, (unsigned)bid, last.pos[0], last.pos[1], last.pos[2]);
        printf("             delivered=%.3f  C=%.3f  tilling=%.3f  frag=%d below=%d\n",
               last.vel[0], last.vel[1], last.vel[2],
               (int)(last.flags & 1u), (int)((last.flags >> 1) & 1u));
        printf("             file frame %zu B  live orb %zu B  (both 64 — not a 128 B Forge line)\n",
               sizeof(last), sizeof(ZtpMycelialState));
    } else {
        printf("  mycelial_terminal.soma.bin missing after write\n");
        return 1;
    }

    printf("  held — 10 Hz on hypha.c; SPECTRA 8 x f64 wired; not on machine.c\n");
    return 0;
}
