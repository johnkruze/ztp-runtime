//! Plasma sheath vs GPS L1 — same law as genesis_core `physics/aero.rs`
//! and `hypersonic_plasma_blackout` (20 Hz, DT=0.05).
//! f_p ≈ 8.98√n_e. Blackout when f_p > 1.57542 GHz.
//! During blackout the filter does not see true velocity. Last GPS freezes.
//! Fin lock latches after 1.5 s continuous blackout.
//! Ablation / Sutton-Graves / HGV 1 kHz Euler do not live here.

pub const GPS_L1_HZ: f64 = 1.57542e9;
pub const A_SL: f64 = 340.3;
pub const DECK_HALF_M: f64 = 39.0;
pub const FIN_LOCK_S: f64 = 1.5;
pub const DT_PLASMA: f64 = 0.05;

/// Electron plasma frequency [Hz]. f_p ≈ 8.98 √n_e  with n_e in m⁻³.
#[inline]
pub fn plasma_frequency_hz(electron_density_m3: f64) -> f64 {
    8.98 * electron_density_m3.max(0.0).sqrt()
}

#[inline]
pub fn gps_l1_blackout(electron_density_m3: f64) -> bool {
    plasma_frequency_hz(electron_density_m3) > GPS_L1_HZ
}

/// Named sheath density [m⁻³]. Not Saha.
/// Peak at 22 km. Amplitude ∝ ((M−2.8)/3)³. Mach ≲ 3.2 is quiet.
#[inline]
pub fn sheath_electron_density_m3(mach: f64, altitude_m: f64) -> f64 {
    let m_ex = (mach - 2.8).max(0.0) / 3.0;
    let n_ref = 2.5e18 * m_ex.powi(3);
    let layer = (-((altitude_m - 22_000.0) / 9_000.0).powi(2)).exp();
    n_ref * layer
}

#[inline]
pub fn tas_from_mach(mach: f64) -> f64 {
    mach.max(0.0) * A_SL
}

/// Live plasma dive. Not a 64-byte last-state frame. Not the 128 B HGV cache.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct C_PlasmaState {
    pub mach: f64,
    pub altitude_m: f64,
    pub x_m: f64,
    pub x_est_m: f64,
    pub tgt_m: f64,
    pub vx_m_s: f64,
    pub vz_m_s: f64,
    pub v_tgt_m_s: f64,
    pub t_black_s: f64,
    pub peak_fp_hz: f64,
    pub miss_m: f64,
    pub fin_lock: bool,
    pub saw_blackout: bool,
    pub impacted: bool,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct C_PlasmaResult {
    pub n_e_m3: f64,
    pub fp_hz: f64,
    pub l1_hz: f64,
    pub fp_over_l1: f64,
    pub miss_m: f64,
    pub blackout: bool,
    pub is_miss: bool,
    pub gps_held: bool,
}

const _: () = {
    assert!(core::mem::size_of::<C_PlasmaState>() >= 88);
    assert!(core::mem::size_of::<C_PlasmaResult>() >= 40);
};

impl C_PlasmaState {
    pub fn init(mach: f64, z0_m: f64, dive_rad: f64, v_tgt_m_s: f64) -> Self {
        let v = tas_from_mach(mach);
        let vz = -v * dive_rad.sin();
        let vx = v * dive_rad.cos();
        let t_impact = z0_m / vz.abs().max(1e-9);
        let tgt = t_impact * vx;
        Self {
            mach,
            altitude_m: z0_m,
            x_m: 0.0,
            x_est_m: tgt,
            tgt_m: tgt,
            vx_m_s: vx,
            vz_m_s: vz,
            v_tgt_m_s: v_tgt_m_s,
            t_black_s: 0.0,
            peak_fp_hz: 0.0,
            miss_m: 0.0,
            fin_lock: false,
            saw_blackout: false,
            impacted: false,
        }
    }
}

/// One 20 Hz tick. Matches `hypersonic_plasma_blackout` inner loop.
pub fn fp_vs_l1(state: &mut C_PlasmaState, dt: f64) -> C_PlasmaResult {
    let dt = if dt > 0.0 { dt } else { DT_PLASMA };
    if state.impacted {
        let gps_held = !state.saw_blackout && state.miss_m <= DECK_HALF_M;
        return C_PlasmaResult {
            n_e_m3: 0.0,
            fp_hz: 0.0,
            l1_hz: GPS_L1_HZ,
            fp_over_l1: 0.0,
            miss_m: state.miss_m,
            blackout: state.saw_blackout,
            is_miss: state.miss_m > DECK_HALF_M,
            gps_held,
        };
    }

    state.altitude_m += state.vz_m_s * dt;
    state.tgt_m += state.v_tgt_m_s * dt;
    let z = state.altitude_m.max(0.0);
    let n_e = sheath_electron_density_m3(state.mach, z);
    let fp = plasma_frequency_hz(n_e);
    if fp > state.peak_fp_hz {
        state.peak_fp_hz = fp;
    }
    let black = state.altitude_m > 0.0 && gps_l1_blackout(n_e);
    if black {
        state.saw_blackout = true;
        state.t_black_s += dt;
        if state.t_black_s > FIN_LOCK_S {
            state.fin_lock = true;
        }
    } else if !state.fin_lock {
        state.x_est_m = state.tgt_m;
        state.t_black_s = 0.0;
    }
    let tti = (z / state.vz_m_s.abs().max(1e-9)).max(0.05);
    let vx_cmd = if state.fin_lock {
        state.vx_m_s
    } else {
        (state.x_est_m - state.x_m) / tti
    };
    state.x_m += vx_cmd * dt;
    if state.altitude_m <= 0.0 {
        state.miss_m = (state.x_m - state.tgt_m).abs();
        state.altitude_m = 0.0;
        state.impacted = true;
    }

    let is_miss = state.impacted && state.miss_m > DECK_HALF_M;
    let gps_held = state.impacted && !state.saw_blackout && !is_miss;
    C_PlasmaResult {
        n_e_m3: n_e,
        fp_hz: fp,
        l1_hz: GPS_L1_HZ,
        fp_over_l1: fp / GPS_L1_HZ,
        miss_m: state.miss_m,
        blackout: black || state.saw_blackout,
        is_miss,
        gps_held,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plasma_l1_cutoff() {
        assert!(!gps_l1_blackout(1.0e16));
        assert!(gps_l1_blackout(1.0e18));
        let n = sheath_electron_density_m3(6.0, 22_000.0);
        assert!(gps_l1_blackout(n));
        let n_high = sheath_electron_density_m3(2.5, 22_000.0);
        assert!(!gps_l1_blackout(n_high));
    }

    #[test]
    fn low_mach_never_blacks() {
        let mut s = C_PlasmaState::init(2.5, 22_000.0, 45.0_f64.to_radians(), 12.0);
        let mut saw = false;
        for _ in 0..2000 {
            let r = fp_vs_l1(&mut s, DT_PLASMA);
            if r.blackout {
                saw = true;
            }
            if s.impacted {
                break;
            }
        }
        assert!(!saw);
        assert!(s.impacted);
        assert!(s.miss_m <= DECK_HALF_M);
    }
}
