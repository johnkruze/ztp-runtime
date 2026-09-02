/* ztp-runtime C ABI. One dylib, many clocks — see examples/Makefile (`make help`).
   Layouts match dexterous.rs, compounding.rs, marine.rs, bluerov.rs, drone.rs,
   last_state.rs / SOMA.md, orbital, mars, tokamak, swing, plasma, mycelial, vehicle, tesseract. */
#ifndef ZTP_H
#define ZTP_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float normal;
    float shear_x;
    float shear_y;
} ZtpTaxel;

typedef struct {
    ZtpTaxel taxels[16];
} ZtpTactileArray;

typedef struct {
    float normal_force;
    float slip_velocity;
    float slip_angular_velocity;
    float object_mass;
    float static_friction_coeff;
    float dynamic_friction_coeff;
    bool reflex_active;
} ZtpGraspState;

typedef struct {
    bool micro_slip_detected;
    bool macro_slip_detected;
    bool rotational_slip_detected;
    float commanded_force;
    float margin;
    float estimated_mu;
} ZtpGraspResult;

ZtpGraspResult ztp_dexterous_evaluate_grasp(
    const ZtpTactileArray *sensor,
    ZtpGraspState *state,
    float dt
);

/* Hand — serial phalanges + tendon placing five pads into the same cone.
   Thumb opposition. Dual-regime: tendon overstretch vs pad slip. dt = 0.001. */
typedef struct {
    float q_mcp[5];
    float q_pip[5];
    float q_dip[5];
    float qdot_mcp[5];
    float qdot_pip[5];
    float qdot_dip[5];
    float tendon_stretch_m;
    float tendon_tension_n;
    float opposition_rad;
    float object_span_m;
    float commanded_close_rad;
    float pad_normal_n;
    float normal_force;
    float slip_velocity;
    float slip_angular_velocity;
    float object_mass;
    float static_friction_coeff;
    float dynamic_friction_coeff;
    bool reflex_active;
} ZtpHandTendonState;

typedef struct {
    bool tendon_overstretch;
    bool pad_slip;
    float commanded_force;
    float margin;
    float tendon_tension_n;
    float pad_normal_n;
    float stretch_m;
    float strain;
} ZtpHandTendonResult;

ZtpHandTendonResult ztp_dexterous_evaluate_hand(
    ZtpHandTendonState *state,
    float dt
);

/* Surgical clamp — do not destroy the sample. Matches dexterous.rs C ABI. */
typedef struct {
    uint32_t tissue_type_id;        /* 0 liver/spleen 1.2 N · 1 bowel/vessel 2.5 N · 2 bone/tendon 40 N */
    float max_tearing_force_n;
    float measured_displacement_m;
    float measured_force_n;
    float relaxation_tau;
    float last_displacement_m;
    float last_force_n;
    float accumulated_energy_j;
} ZtpSurgicalTissueAuditor;

typedef struct {
    bool tissue_overstress_detected;
    bool viscoelastic_rupture_detected;
    bool cable_slip_fault;
    float clamped_force;
} ZtpSurgicalResult;

ZtpSurgicalResult ztp_surgical_evaluate_grasp(
    const ZtpSurgicalTissueAuditor *auditor,
    float dt
);

/* Micro-assembly release — do not smash the part on retract.
   Stiction: jaw>10 µm and pull-off>5 µN. ESD: charge>150 V. Piezo tracks stiction. */
typedef struct {
    float part_mass_micrograms;
    float pull_off_force_un;
    float jaw_separation_um;
    float dynamic_electrostatic_charge_v;
    float last_jaw_separation_um;
} ZtpMicroReleaseAuditor;

typedef struct {
    bool release_stiction_active;
    bool electrostatic_charge_violation;
    bool piezo_shake_trigger;
    bool safe_to_retract;
} ZtpMicroResult;

ZtpMicroResult ztp_micro_evaluate_release(
    const ZtpMicroReleaseAuditor *auditor,
    float dt
);

/* Ostwald–de Waele mill — same dylib. Instantaneous η = K γ̇^{n-1}.
   Audit is accumulated shear vs the named gate (broth 15 Pa default in the crate).
   Layout matches compounding.rs #[repr(C, align(128))]. */
typedef struct {
    float consistency_index_k;
    float flow_index_n;
    float critical_shear_limit;
    float accumulated_shear_stress;
} __attribute__((aligned(128))) ZtpOstwaldFluid;

float ztp_compounding_compute_viscosity(
    ZtpOstwaldFluid *fluid,
    float shear_rate
);

bool ztp_compounding_audit_shear(
    const ZtpOstwaldFluid *fluid
);

/* Body-coherence + last-state seal — same dylib, surgical machine's other engines.
   Layout matches compounding.rs #[repr(C, align(128))]. */
typedef struct {
    float heart_rate_bpm;
    float respiratory_frequency_hz;
    float trigeminal_pressure_pa;
    float autonomic_ratio;
} __attribute__((aligned(128))) ZtpVagalBridge;

typedef struct {
    uint64_t timestamp_ms;
    float pathogen_concentration;
    float cytoplasmic_viscosity;
    float mucosal_dissolution_rate;
    float autonomic_ratio;
    uint8_t hash_seal[32];
} __attribute__((aligned(128))) ZtpBiologicalState;

float ztp_compounding_update_autonomic_tone(
    ZtpVagalBridge *bridge,
    float mechanical_chewing_pa,
    float diaphragmatic_pressure_pa
);

void ztp_compounding_seal_state(
    ZtpBiologicalState *state,
    uint8_t *out_hash
);

/* Marine last-state — Mackenzie 1981, hydrostatic ρ g z, thermocline Snell.
   8 × f64 = 64 bytes, same orb as SPECTRA OceanState. Do not pad. */
typedef struct {
    double depth_m;
    double velocity_ms;
    double buoyancy_n;
    double pressure_pa;
    double sound_speed_ms;
    double pitch_rad;
    double dc_dz;
    double seal_f64;
} ZtpMarineState;

_Static_assert(sizeof(ZtpMarineState) == 64, "ZtpMarineState is 8 x f64");

ZtpMarineState ztp_marine_evaluate_state(float depth_m, float time_step);

/* BlueROV — DVL-lost hydro coherence vs IMU. Same dylib, GPS-denied body in the column. */
typedef struct {
    double position[3];
    double velocity[3];
    double pitch_roll_yaw[3];
    double thruster_commands[6];
    double current_velocity[3];
    double mass;
    double volume;
    double drag_coefficients[3];
    double max_thrust_horizontal;
    double max_thrust_vertical;
    double tether_anchor[3];
    double tether_length;
    double tether_k;
} ZtpBlueRovState;

typedef struct {
    double imu_acceleration[3];
    double true_acceleration[3];
    double coherence_residual;
    bool coherence_fail;
} ZtpBlueRovResult;

ZtpBlueRovResult ztp_bluerov_step(
    ZtpBlueRovState *state,
    double nav_vel_x,
    double nav_vel_y,
    double nav_vel_z,
    double coherence_threshold,
    double dt
);

/* Drone — VSLAM vs IMU coherence. Same dylib, GPS-denied body in air. */
typedef struct {
    double position[3];
    double velocity[3];
    double pitch_roll_yaw[3];
    double motor_rpm[4];
    double wind_velocity[3];
    double mass;
    double drag_coefficient;
    double max_thrust;
} ZtpDroneState;

typedef struct {
    double imu_acceleration[3];
    double true_acceleration[3];
    double coherence_residual;
    bool coherence_fail;
} ZtpDroneResult;

ZtpDroneResult ztp_drone_step(
    ZtpDroneState *state,
    double vslam_vel_x,
    double vslam_vel_y,
    double vslam_vel_z,
    double vslam_vel_prev_x,
    double vslam_vel_prev_y,
    double vslam_vel_prev_z,
    double coherence_threshold,
    double dt
);

/* Last-state *file* pinout — 64 B header + N × 64 B frames.
   Matches genesis_core::last_state and grokd/public/soma/SOMA.md.
   Magic is in the header only. Marine 8×f64 above is a live ocean orb, not this file. */
typedef struct {
    char magic[4];
    uint16_t spec_version;
    uint16_t body_id;           /* 6 ocean · 7 drone · 9 vehicle · 10 plasma · 11 fusion · 27 autolab · 28 grasp · 30 humanoid · 31 hand · 32 compounding */
    uint64_t traj_count;
    uint64_t frame_count;
    uint8_t digest[32];
    char reserved[8];
} ZtpLastStateHeader;

typedef struct {
    double t;
    float pos[3];
    float vel[3];
    float force_torque;
    float residual;
    uint64_t flags;             /* body 6: crushed / starved · 7: dark / vslam / reflex · 9: hydroplane / corner-lost / grip · 28/31: overstretch / slip · 30: dark / buckle / reflex · 32: potency-collapsed / dissolution-stalled */
    uint8_t proof[16];
} ZtpLastStateFrame;

_Static_assert(sizeof(ZtpLastStateHeader) == 64, "ZtpLastStateHeader is 64 B");
_Static_assert(sizeof(ZtpLastStateFrame) == 64, "ZtpLastStateFrame is 64 B");

bool ztp_last_state_header_ok(const uint8_t *file, uint64_t len);
bool ztp_last_state_peek_last(const uint8_t *file, uint64_t len, ZtpLastStateFrame *out);
uint16_t ztp_last_state_body_id(const uint8_t *header);
ZtpLastStateFrame ztp_last_state_pack_humanoid(
    uint32_t timestamp_ms,
    float com_x, float com_y, float com_z,
    float vel_x, float vel_y, float vel_z,
    float pitch_rad,
    float zmp_margin_m,
    bool is_dark_window,
    bool is_buckle,
    bool is_reflex_grasp
);

ZtpLastStateFrame ztp_last_state_pack_hand(
    uint32_t timestamp_ms,
    float tension_n,
    float pad_normal_n,
    float stretch_m,
    float opposition_rad,
    float q_mcp,
    float slip_m_s,
    float margin,
    float object_span_m,
    bool tendon_overstretch,
    bool pad_slip
);

ZtpLastStateFrame ztp_last_state_pack_ocean(
    uint32_t timestamp_ms,
    float max_depth_m,
    float peak_pressure_mpa,
    float battery_wh,
    float true_crush_m,
    float believed_crush_m,
    float battery_used_pct,
    float mass_kg,
    float target_depth_m,
    bool is_crushed,
    bool is_power_starved
);

ZtpLastStateFrame ztp_last_state_pack_drone(
    uint32_t timestamp_ms,
    float pos_x, float pos_y, float pos_z,
    float vel_x, float vel_y, float vel_z,
    float pitch_rad,
    float coherence_residual,
    bool is_dark_window,
    bool is_vslam_fail,
    bool is_reflex_active
);

/* Aetheric handshake — Friis / Shannon / hop seed. Same dylib. Link when the radio works. */
typedef struct {
    bool success;
    double resonance;
    double avg_snr_db;
} ZtpHandshakeResult;

ZtpHandshakeResult ztp_atheric_handshake(
    const uint8_t *seed_bytes,
    double strength,
    double distance_km
);

/* Orbital Dark Window — 6DOF + attitude. Same dylib. Clock on orbit.c is 100 Hz (dt=0.01).
   Layout matches lib.rs C_SatelliteState. Do not tick this on machine.c. */
typedef struct {
    double position[3];
    double velocity[3];
    double quaternion_attitude[4];
    double angular_velocity[3];
    double inertia_tensor[9];
} ZtpSatelliteState;

void ztp_orbital_step_6dof(ZtpSatelliteState *state, double dt);
void ztp_orbital_step_attitude(
    ZtpSatelliteState *state,
    double ext_torque_x,
    double ext_torque_y,
    double ext_torque_z,
    double dt
);

/* Mars CO₂ EDL — same dylib. Integrator dt=0.001. Live in orbit.c as a second clock,
   not on the grasp 1 kHz loop. Layout matches lib.rs C_MarsState / C_MarsResult. */
typedef struct {
    double position[3];
    double velocity[3];
    double dry_mass;
    double drag_area;
    double cd;
    double fuel_mass;
    double specific_impulse;
} ZtpMarsState;

typedef struct {
    double density;
    double drag_force[3];
    double net_accel[3];
} ZtpMarsResult;

ZtpMarsResult ztp_mars_step(
    ZtpMarsState *state,
    double retro_thrust,
    double dt
);

/* STREAM4 fusion — tokamak dt=1 µs (confine.c). Swing dt=1 ms (grid.c).
   Not machine.c. Reactor xenon pit is hours: terminal last-state only. */
typedef struct {
    double plasma_radius;
    double radial_velocity;
    double z_displacement;
    double z_velocity;
    double temperature;
    double particle_density;
    double b_field;
    double b_field_asymmetry;
    double pf_voltage;
    double pf_current;
    double coil_temp;
    double residual;
    uint64_t time_us;
    bool coil_quenched;
    bool quenched;
} ZtpTokamakState;

void ztp_tokamak_step(ZtpTokamakState *state, double dt_us);
double ztp_tokamak_equilibrium_b(const ZtpTokamakState *state);
void ztp_tokamak_apply_ai_field(
    ZtpTokamakState *state,
    double target_b,
    double radial_noise,
    double z_asymmetry_noise
);

typedef struct {
    double h_constant;
    double damping;
    double p_mech;
    double p_max;
    double delta;
    double delta_omega;
    double inverter_fraction;
    double pll_err;
    double governor_valve;
    double residual;
    uint64_t time_ms;
    bool inverter_tripped;
    bool cascaded;
} ZtpSwingState;

void ztp_swing_step(ZtpSwingState *state, double dt);
void ztp_swing_weather_loss(ZtpSwingState *state, double loss_percentage);
void ztp_swing_ai_mismatch(ZtpSwingState *state, double mismatch_percentage);

ZtpLastStateFrame ztp_last_state_pack_fusion(
    double t_s,
    float flux,
    float beta_eff,
    float time_s,
    float xenon_worth,
    float pit_hours,
    float core_age_days,
    float delta_rho,
    float base_rho,
    bool is_prompt_critical,
    bool is_pit_survived
);

/* STREAM3 reentry / plasma — 20 Hz (dt=0.05). GPS L1 vs f_p. Not HGV Euler.
   Not machine.c. Ablation does not tick here. File pinout is LastStateFrame64. */
typedef struct {
    double mach;
    double altitude_m;
    double x_m;
    double x_est_m;
    double tgt_m;
    double vx_m_s;
    double vz_m_s;
    double v_tgt_m_s;
    double t_black_s;
    double peak_fp_hz;
    double miss_m;
    bool fin_lock;
    bool saw_blackout;
    bool impacted;
} ZtpPlasmaState;

typedef struct {
    double n_e_m3;
    double fp_hz;
    double l1_hz;
    double fp_over_l1;
    double miss_m;
    bool blackout;
    bool is_miss;
    bool gps_held;
} ZtpPlasmaResult;

void ztp_plasma_init(
    ZtpPlasmaState *state,
    double mach,
    double z0_m,
    double dive_rad,
    double v_tgt_m_s
);
ZtpPlasmaResult ztp_plasma_fp_vs_l1(ZtpPlasmaState *state, double dt);

ZtpLastStateFrame ztp_last_state_pack_plasma(
    uint32_t timestamp_ms,
    float last_gps_x_m,
    float altitude_m,
    float last_gps_tgt_m,
    float fp_ghz,
    float l1_ghz,
    float miss_m,
    float fp_over_l1,
    float miss_repeat_m,
    bool is_blackout,
    bool is_miss,
    bool is_gps_held
);

uint64_t ztp_last_state_write(
    uint16_t body_id,
    const char *reserved8,
    const ZtpLastStateFrame *frames,
    uint64_t n_frames,
    uint8_t *out,
    uint64_t out_cap
);

/* STREAM5 compounding mill — body 32 BROTH001. Same dylib. Peek on machine.c. */
ZtpLastStateFrame ztp_last_state_pack_compounding(
    uint32_t timestamp_ms,
    float accumulated_shear_stress_pa,
    float active_potency_pct,
    float dissolution_pct,
    float final_viscosity_pas,
    float final_api_concentration_kg_m3,
    float shear_rate_s1,
    bool is_potency_collapsed,
    bool is_dissolution_stalled
);

/* STREAM1 mycelial — SPECTRA MycelialState is already 8×f64 = 64 B in
   spectra_genesis terran.rs. Wire it. Do not redesign the orb. Clock 10 Hz
   on hypha.c (dt=0.1). Not machine.c. */
typedef struct {
    double health_index;
    double hyphal_density;
    double percolation_index;
    double delivered_nutrient;
    double fragmented_flag;
    double conductance_mean;
    double tilling_stress;
    double seal_f64;
} ZtpMycelialState;

_Static_assert(sizeof(ZtpMycelialState) == 64, "ZtpMycelialState is 8 x f64");

ZtpMycelialState ztp_mycelial_evaluate_state(
    float health_index,
    float hyphal_density,
    float tilling_stress,
    float time_s
);

ZtpLastStateFrame ztp_last_state_pack_mycelial(
    uint32_t timestamp_ms,
    float health_index,
    float hyphal_density,
    float percolation_index,
    float delivered_nutrient,
    float conductance_mean,
    float tilling_stress,
    bool is_fragmented,
    bool is_below_percolation
);

bool ztp_last_state_write_mycelial(
    const char *path,
    const ZtpLastStateFrame *frames,
    uint64_t n_frames
);

/* STREAM2 vehicle — Pacejka hydroplane 1 kHz. chassis.c, not machine.c.
   ztp_terran_evaluate_contact is soil, not this car.
   128 B Forge VehicleDynamicsState cache line ≠ 64 B LastStateFrame64 file. */
typedef struct {
    float timestamp;
    float chassis_q[6];
    float chassis_dq[6];
    float wheel_q[4];
    float wheel_dq[4];
    float wheel_torques[4];
    float pacejka_jacobians[4];
    float normal_forces[2];
    float thermal_accumulated;
} __attribute__((aligned(128))) ZtpVehicleDynamicsState;

_Static_assert(sizeof(ZtpVehicleDynamicsState) == 128, "ZtpVehicleDynamicsState is 128 B Forge cache line");

typedef struct {
    float mu;
    float abs_y_m;
    float yaw_rad;
    bool hydroplane;
    bool corner_lost;
    bool grip;
} ZtpVehicleHydroplaneResult;

ZtpVehicleHydroplaneResult ztp_vehicle_hydroplane_step(
    ZtpVehicleDynamicsState *state,
    float mu_dry,
    float mu_wet,
    float x_water_m,
    float steer_rad,
    float mass_kg,
    float v_hold_ms,
    float dt
);

ZtpLastStateFrame ztp_last_state_pack_vehicle(
    uint32_t timestamp_ms,
    float mu,
    float abs_y_m,
    float yaw_rad,
    float vel_x,
    float vel_y,
    float yaw_rate,
    bool is_hydroplane,
    bool is_corner_lost,
    bool is_grip
);

ZtpLastStateFrame ztp_last_state_pack_tesseract(
    double t_s,
    float peak_disp_m,
    float well_velocity_m_s,
    float drive_velocity_m_s,
    float alpha,
    float bias_m_s2,
    float omega_ext_rad_s,
    float scale_factor_n,
    float bias_floor_m,
    bool is_nonlinear_drive,
    bool is_bias_floor_broken
);

/* Tesseract IMU firewall. Host dt=0.001 (chassis). Resonator ω_n is 100 Hz.
   Not machine.c. Body 12 last-state file is tesseract_terminal.soma.bin (this sitting).
   Live orb ZtpTesseractState remains 8×f64 RAM — not the file.
   Bias enters the Euler (a + b + u + spring + damp + cubic).
   drive_velocity_m_s is sense-axis speed for F_c = 2 m |v| |Ω|, not well ẋ.
   PHYSICAL_ANOMALY = nonlinear drive AND bias-floor broken. */
#define ZTP_OK 0
#define ZTP_PHYSICAL_ANOMALY 1

typedef struct {
    double displacement_m;
    double velocity_m_s;
    double mass_kg;
    double omega_n_rad_s;
    double zeta;
    double alpha;
    double control_u;
    double time_s;
} ZtpTesseractState;

_Static_assert(sizeof(ZtpTesseractState) == 64, "ZtpTesseractState is 8 x f64");

typedef struct {
    double displacement_m;
    double velocity_m_s;
    double scale_factor_n;
    double residual;
    double bias_floor_m;
    bool hold_ok;
    bool nonlinear;
    bool bias_floor_broken;
} ZtpTesseractTick;

_Static_assert(sizeof(ZtpTesseractTick) == 48, "ZtpTesseractTick is 5 x f64 + 3 bool");

int32_t ztp_tesseract_step(
    ZtpTesseractState *state,
    double inertial_accel,
    double omega_ext_rad_s,
    double x_cmd_m,
    double v_cmd_m_s,
    double k_p,
    double k_d,
    double bias_m_s2,
    double drive_velocity_m_s,
    double dt,
    ZtpTesseractTick *tick
);

#ifdef __cplusplus
}
#endif
#endif
