//! Marine last-state organ — Mackenzie 1981, hydrostatic ρ g z, thermocline Snell.
//! Same vibration as genesis_core `physics/marine.rs` and SPECTRA `OceanState`.
//! Layout is 8 × f64 = 64 B. Do not align(128): that would break the orb.

/// SPECTRA / last-state ocean frame. Named slots, 64 bytes.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct C_MarineState {
    pub depth_m: f64,
    pub velocity_ms: f64,
    pub buoyancy_n: f64,
    pub pressure_pa: f64,
    pub sound_speed_ms: f64,
    pub pitch_rad: f64,
    pub dc_dz: f64,
    pub seal_f64: f64,
}

const _: () = {
    assert!(core::mem::size_of::<C_MarineState>() == 64);
    assert!(core::mem::align_of::<C_MarineState>() == 8);
    assert!(core::mem::offset_of!(C_MarineState, seal_f64) == 56);
};

pub const RHO_SEAWATER: f64 = 1025.0;
pub const GRAVITY: f64 = 9.81;
pub const SEA_TEMP_C: f64 = 18.0;
pub const SALINITY_PSU: f64 = 35.0;
pub const T_DEEP_C: f64 = 4.0;
pub const THERMOCLINE_THICKNESS_M: f64 = 40.0;
pub const DESCENT_MS: f64 = 1.5;
pub const DISPLACEMENT_M3: f64 = 1.0;
pub const GM_M: f64 = 0.80;
pub const K_GYRATION_M: f64 = 1.35;

/// Mackenzie 1981 sound speed [m/s]. T in °C, S in psu, z depth in m.
pub fn mackenzie_sound_speed(temp_c: f64, salinity_psu: f64, depth_m: f64) -> f64 {
    let t = temp_c;
    let d = depth_m;
    let ds = salinity_psu - 35.0;
    1448.96 + 4.591 * t - 5.304e-2 * t * t + 2.374e-4 * t * t * t
        + 1.340 * ds
        + 1.630e-2 * d
        + 1.675e-7 * d * d
        - 1.025e-2 * t * ds
        - 7.139e-13 * t * d * d * d
}

/// Snell circular-ray radius R = −c / (dc/dz). Positive R bends the ray downward.
pub fn acoustic_ray_radius(sound_speed: f64, dc_dz: f64) -> f64 {
    if dc_dz.abs() < 1e-12 {
        return if dc_dz >= 0.0 {
            f64::INFINITY
        } else {
            f64::NEG_INFINITY
        };
    }
    -sound_speed / dc_dz
}

/// Small-angle roll natural frequency [rad/s]: ω = √(g GM / k²).
pub fn roll_natural_omega(gm_m: f64, k_gyration_m: f64) -> f64 {
    (GRAVITY * gm_m.max(1e-9) / k_gyration_m.max(1e-6).powi(2)).sqrt()
}

fn seal_from_slots(slots: [f64; 7]) -> f64 {
    let mut h: u64 = 0x4D41_434B_3139_3831;
    for x in slots {
        h ^= x.to_bits();
        h = h.rotate_left(13).wrapping_mul(0x9E37_79B9_7F4A_7C15);
    }
    f64::from_bits((0x3FF_u64 << 52) | (h & 0x000F_FFFF_FFFF_FFFF))
}

/// One tick of the ocean last-state organ. Matches SPECTRA `ocean::run_simulation`.
pub fn evaluate_state(depth: f64, time_step: f64) -> C_MarineState {
    let depth_m = depth + DESCENT_MS * time_step;
    let velocity_ms = DESCENT_MS;
    let buoyancy_n = RHO_SEAWATER * GRAVITY * DISPLACEMENT_M3;
    let pressure_pa = RHO_SEAWATER * GRAVITY * depth_m;
    let sound_speed_ms = mackenzie_sound_speed(SEA_TEMP_C, SALINITY_PSU, depth_m);
    let c_upper = sound_speed_ms;
    let c_lower = mackenzie_sound_speed(
        T_DEEP_C,
        SALINITY_PSU,
        depth_m + THERMOCLINE_THICKNESS_M,
    );
    let dc_dz = (c_lower - c_upper) / THERMOCLINE_THICKNESS_M;
    let _ray_radius_m = acoustic_ray_radius(sound_speed_ms, dc_dz);
    let _omega_n = roll_natural_omega(GM_M, K_GYRATION_M);
    let pitch_rad = 0.0;
    let seal_f64 = seal_from_slots([
        depth_m,
        velocity_ms,
        buoyancy_n,
        pressure_pa,
        sound_speed_ms,
        pitch_rad,
        dc_dz,
    ]);
    C_MarineState {
        depth_m,
        velocity_ms,
        buoyancy_n,
        pressure_pa,
        sound_speed_ms,
        pitch_rad,
        dc_dz,
        seal_f64,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn marine_state_is_64_bytes() {
        assert_eq!(core::mem::size_of::<C_MarineState>(), 64);
        assert_eq!(core::mem::offset_of!(C_MarineState, seal_f64), 56);
    }

    #[test]
    fn mackenzie_named_sea_surface() {
        let c = mackenzie_sound_speed(18.0, 35.0, 0.0);
        assert!((c - 1515.7975568).abs() < 1e-7);
    }

    #[test]
    fn evaluate_fills_mackenzie_and_thermocline() {
        let s = evaluate_state(100.0, 1.0);
        assert!((s.depth_m - 101.5).abs() < 1e-12);
        assert!((s.pressure_pa - RHO_SEAWATER * GRAVITY * 101.5).abs() < 1e-6);
        assert!((s.sound_speed_ms - 1500.0).abs() > 1.0);
        assert!(s.dc_dz < 0.0 && s.dc_dz.is_finite());
        let r = acoustic_ray_radius(s.sound_speed_ms, s.dc_dz);
        assert!(r.is_finite() && r > 0.0);
    }
}
