/* Named sea. Mackenzie 1981. Not 1500. Not machine.c.
   18 C, 35 psu. make && ./ocean */
#include <stdio.h>
#include "../../include/ztp.h"

int main(void) {
    _Static_assert(sizeof(ZtpMarineState) == 64, "ZtpMarineState is 8 x f64");
    const float dt = 0.001f;
    const float z0 = 100.0f;
    ZtpMarineState s = ztp_marine_evaluate_state(z0, dt);

    printf("ztp-runtime eval  ·  ocean  ·  Mackenzie 1981  ·  named sea 18 C 35 psu\n");
    printf("  sizeof(ZtpMarineState)=%zu  (8 x f64 live orb, not a .soma.bin)\n",
           sizeof(ZtpMarineState));
    printf("  z=%.4f m  c=%.3f m/s  P=%.3f Pa  dc/dz=%.6f\n",
           s.depth_m, s.sound_speed_ms, s.pressure_pa, s.dc_dz);

    int not_costume = s.sound_speed_ms > 1510.0 && s.sound_speed_ms < 1525.0
        && s.dc_dz < 0.0;
    printf("  %s\n", not_costume
           ? "held - named sea, not 1500 costume"
           : "check: sound speed left Mackenzie class");
    return not_costume ? 0 : 1;
}
