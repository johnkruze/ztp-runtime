//! Tesseract IMU firewall. Duffing drive + Coriolis scale + PD hold.
//!
//! Host dt = 0.001 (chassis firewall). Resonator clock is ω_n (100 Hz on
//! the sealed bank). This is not a 1000 Hz constitutive look.
//! Not machine.c. Body 12 last-state is not this sitting.
//!
//! Forge twin: `physics/tesseract.rs`. Same Euler so α = 0 is Cluster D.
//! Bias enters the well: total_accel = a + b + u + spring + damp + cubic.
//! `drive_velocity_m_s` is sense-axis speed for F_c, not well ẋ.
//! PHYSICAL_ANOMALY is nonlinear drive AND bias-floor broken. Reject —
//! do not write the state.

pub const ZTP_OK: i32 = 0;
pub const ZTP_PHYSICAL_ANOMALY: i32 = 1;

/// Linear-plateau proof-mass velocity [m/s].
pub const V_LINEAR_M_S: f64 = 1.0;
/// High-velocity tether [m/s]. Scale factor ∝ m v.
pub const V_TETHER_M_S: f64 = 70.0;
/// Tether gate — same 0.85× mix as the sealed bank.
pub const TETHER_GATE_M_S: f64 = V_TETHER_M_S * 0.85;
/// Accel-bias position lock-loss [m]. ½ |b| t².
pub const LOCK_LOSS_M: f64 = 0.05;
/// Control saturation [m/s²].
pub const HOLD_U_SAT_M_S2: f64 = 800.0;

/// 8 × f64 = 64 B live orb. Not a last-state pack. Do not steal body 12.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct C_TesseractState {
    pub displacement_m: f64,
    pub velocity_m_s: f64,
    pub mass_kg: f64,
    pub omega_n_rad_s: f64,
    pub zeta: f64,
    /// Duffing cubic [1/m² s²]. Zero recovers the linear oscillator.
    pub alpha: f64,
    pub control_u: f64,
    /// Body time [s] for the reduced-order bias floor.
    pub time_s: f64,
}

const _: () = {
    assert!(core::mem::size_of::<C_TesseractState>() == 64);
    assert!(core::mem::align_of::<C_TesseractState>() == 8);
};

/// One accepted (or rejected) tick. Stack only.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct C_TesseractTick {
    pub displacement_m: f64,
    pub velocity_m_s: f64,
    /// Instantaneous Coriolis |F_c| = 2 m |v_sense| |Ω| [N].
    /// `v_sense` is the drive_velocity_m_s argument (tether or plateau).
    pub scale_factor_n: f64,
    /// Euler lag residual of the drive ODE [m/s²].
    pub residual: f64,
    /// ½ |b| t² [m].
    pub bias_floor_m: f64,
    /// k_p > ω_n².
    pub hold_ok: bool,
    /// α ≠ 0 and |v_drive| ≥ tether gate.
    pub nonlinear: bool,
    /// bias floor ≥ lock-loss.
    pub bias_floor_broken: bool,
}

const _: () = {
    assert!(core::mem::size_of::<C_TesseractTick>() == 48);
};

#[inline]
pub fn coriolis_scale_n(mass_kg: f64, velocity_m_s: f64, omega_rad_s: f64) -> f64 {
    // `velocity_m_s` is sense-axis speed (tether or plateau), not well ẋ.
    2.0 * mass_kg.abs() * velocity_m_s.abs() * omega_rad_s.abs()
}

#[inline]
pub fn control_hold_attractor(
    displacement_m: f64,
    velocity_m_s: f64,
    x_cmd_m: f64,
    v_cmd_m_s: f64,
    k_p: f64,
    k_d: f64,
    u_sat_m_s2: f64,
) -> f64 {
    let u = k_p * (x_cmd_m - displacement_m) + k_d * (v_cmd_m_s - velocity_m_s);
    let sat = u_sat_m_s2.abs();
    if u > sat {
        sat
    } else if u < -sat {
        -sat
    } else {
        u
    }
}

#[inline]
pub fn bias_floor_m(bias_m_s2: f64, t_s: f64) -> f64 {
    let t = if t_s > 0.0 { t_s } else { 0.0 };
    0.5 * bias_m_s2.abs() * t * t
}

#[inline]
pub fn is_nonlinear_drive(alpha: f64, drive_velocity_m_s: f64) -> bool {
    alpha != 0.0 && drive_velocity_m_s.abs() >= TETHER_GATE_M_S
}

#[inline]
pub fn is_bias_floor_broken(bias_m_s2: f64, t_s: f64) -> bool {
    bias_floor_m(bias_m_s2, t_s) >= LOCK_LOSS_M
}

/// One firewall tick. Returns `ZTP_PHYSICAL_ANOMALY` and leaves `state`
/// untouched when nonlinear drive and bias floor are both live.
pub fn step_tesseract(
    state: &mut C_TesseractState,
    inertial_accel: f64,
    omega_ext_rad_s: f64,
    x_cmd_m: f64,
    v_cmd_m_s: f64,
    k_p: f64,
    k_d: f64,
    bias_m_s2: f64,
    drive_velocity_m_s: f64,
    dt: f64,
) -> (i32, C_TesseractTick) {
    let w = if state.omega_n_rad_s > 1e-3 {
        state.omega_n_rad_s
    } else {
        1e-3
    };
    let dt = if dt > 1e-12 { dt } else { 1e-12 };
    let t_end = state.time_s + dt;
    let nonlinear = is_nonlinear_drive(state.alpha, drive_velocity_m_s);
    let floor = bias_floor_m(bias_m_s2, t_end);
    let bias_broken = floor >= LOCK_LOSS_M;
    let hold_ok = k_p > w * w;

    if nonlinear && bias_broken {
        let tick = C_TesseractTick {
            displacement_m: state.displacement_m,
            velocity_m_s: state.velocity_m_s,
            scale_factor_n: coriolis_scale_n(state.mass_kg, drive_velocity_m_s, omega_ext_rad_s),
            residual: 0.0,
            bias_floor_m: floor,
            hold_ok,
            nonlinear,
            bias_floor_broken: true,
        };
        return (ZTP_PHYSICAL_ANOMALY, tick);
    }

    let u = if hold_ok {
        control_hold_attractor(
            state.displacement_m,
            state.velocity_m_s,
            x_cmd_m,
            v_cmd_m_s,
            k_p,
            k_d,
            HOLD_U_SAT_M_S2,
        )
    } else {
        0.0
    };
    state.control_u = u;

    let x = state.displacement_m;
    let v = state.velocity_m_s;
    let spring = -w * w * x;
    let damp = -2.0 * state.zeta * w * v;
    let cubic = -state.alpha * x * x * x;
    let total_accel = inertial_accel + bias_m_s2 + u + spring + damp + cubic;

    state.velocity_m_s = v + total_accel * dt;
    state.displacement_m = state.displacement_m + state.velocity_m_s * dt;
    state.time_s = t_end;

    let actual = (state.velocity_m_s - v) / dt;
    let x_n = state.displacement_m;
    let v_n = state.velocity_m_s;
    let expected_new = inertial_accel + bias_m_s2 + u
        - w * w * x_n
        - 2.0 * state.zeta * w * v_n
        - state.alpha * x_n * x_n * x_n;
    let residual = (actual - expected_new).abs();
    let scale = coriolis_scale_n(state.mass_kg, drive_velocity_m_s, omega_ext_rad_s);

    let tick = C_TesseractTick {
        displacement_m: state.displacement_m,
        velocity_m_s: state.velocity_m_s,
        scale_factor_n: scale,
        residual,
        bias_floor_m: floor,
        hold_ok,
        nonlinear,
        bias_floor_broken: bias_broken,
    };
    (ZTP_OK, tick)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fresh(alpha: f64) -> C_TesseractState {
        C_TesseractState {
            displacement_m: 0.0,
            velocity_m_s: 0.0,
            mass_kg: 1.0e-7,
            omega_n_rad_s: 100.0 * 2.0 * core::f64::consts::PI,
            zeta: 0.05,
            alpha,
            control_u: 0.0,
            time_s: 0.0,
        }
    }

    #[test]
    fn orb_is_64() {
        assert_eq!(core::mem::size_of::<C_TesseractState>(), 64);
    }

    #[test]
    fn alpha_zero_matches_linear_euler() {
        let mut tes = fresh(0.0);
        let mut x = 0.0;
        let mut v = 0.0;
        let w = tes.omega_n_rad_s;
        let zeta = tes.zeta;
        let dt = 0.001;
        for k in 0..200 {
            let t = k as f64 * dt;
            let a = (t * w).sin() * 10.0;
            let (code, _) = step_tesseract(
                &mut tes, a, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, V_LINEAR_M_S, dt,
            );
            assert_eq!(code, ZTP_OK);
            let spring = -w * w * x;
            let damp = -2.0 * zeta * w * v;
            let acc = a + spring + damp;
            v += acc * dt;
            x += v * dt;
            assert!((x - tes.displacement_m).abs() < 1e-12);
            assert!((v - tes.velocity_m_s).abs() < 1e-12);
        }
    }

    #[test]
    fn coriolis_doubles_with_velocity() {
        let m = 1.5e-7;
        let omega = 0.4;
        let s1 = coriolis_scale_n(m, 1.0, omega);
        let s2 = coriolis_scale_n(m, 2.0, omega);
        assert!((s2 - 2.0 * s1).abs() < 1e-18);
    }

    #[test]
    fn hold_ok_is_kp_above_omega_sq() {
        let w = 100.0 * 2.0 * core::f64::consts::PI;
        let w2 = w * w;
        assert!(1.2e6 > w2);
        assert!(12.0 < w2);
        let mut tes = fresh(0.0);
        tes.displacement_m = 0.01;
        let (_, tick_ok) = step_tesseract(
            &mut tes, 0.0, 0.0, 0.0, 0.0, 1.2e6, 1.6e3, 0.01, V_LINEAR_M_S, 0.001,
        );
        assert!(tick_ok.hold_ok);
        let mut tes2 = fresh(0.0);
        tes2.displacement_m = 0.01;
        let (_, tick_lo) = step_tesseract(
            &mut tes2, 0.0, 0.0, 0.0, 0.0, 12.0, 0.0, 0.01, V_LINEAR_M_S, 0.001,
        );
        assert!(!tick_lo.hold_ok);
    }

    #[test]
    fn anomaly_rejects_and_freezes_state() {
        let mut tes = fresh(1.0e9);
        tes.displacement_m = 0.004;
        tes.velocity_m_s = 0.8;
        tes.time_s = 0.40;
        tes.control_u = 12.0;
        let snap = tes;
        let (code, tick) = step_tesseract(
            &mut tes,
            40.0,
            0.5,
            0.01,
            0.0,
            1.2e6,
            1.6e3,
            1.0,
            V_TETHER_M_S,
            0.001,
        );
        assert_eq!(code, ZTP_PHYSICAL_ANOMALY);
        assert!(tick.nonlinear && tick.bias_floor_broken);
        assert_eq!(tes.displacement_m, snap.displacement_m);
        assert_eq!(tes.velocity_m_s, snap.velocity_m_s);
        assert_eq!(tes.control_u, snap.control_u);
        assert_eq!(tes.time_s, snap.time_s);
    }

    #[test]
    fn columns_independent_each_alone_is_ok() {
        // Tether + α, bias in budget.
        let mut a = fresh(1.0e9);
        a.time_s = 0.40;
        let (code_a, tick_a) = step_tesseract(
            &mut a, 10.0, 0.4, 0.0, 0.0, 1.2e6, 1.6e3, 0.05, V_TETHER_M_S, 0.001,
        );
        assert_eq!(code_a, ZTP_OK);
        assert!(tick_a.nonlinear);
        assert!(!tick_a.bias_floor_broken);

        // Plateau, bias floor broken.
        let mut b = fresh(0.0);
        b.time_s = 0.40;
        let (code_b, tick_b) = step_tesseract(
            &mut b, 10.0, 0.4, 0.0, 0.0, 1.2e6, 1.6e3, 1.0, V_LINEAR_M_S, 0.001,
        );
        assert_eq!(code_b, ZTP_OK);
        assert!(!tick_b.nonlinear);
        assert!(tick_b.bias_floor_broken);
    }

    #[test]
    fn bias_in_the_well_moves_displacement() {
        // Plateau, hold off (k_p = 0). Same drive; b ≠ 0 sits the mass
        // at the spring particular solution x ≈ b / ω_n².
        let dt = 0.001;
        let steps = 200;
        let bias = 0.40;
        let mut clean = fresh(0.0);
        let mut biased = fresh(0.0);
        let w = clean.omega_n_rad_s;
        for k in 0..steps {
            let t = k as f64 * dt;
            let a = (t * w).sin() * 10.0;
            let (c0, _) = step_tesseract(
                &mut clean, a, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, V_LINEAR_M_S, dt,
            );
            let (c1, _) = step_tesseract(
                &mut biased, a, 0.0, 0.0, 0.0, 0.0, 0.0, bias, V_LINEAR_M_S, dt,
            );
            assert_eq!(c0, ZTP_OK);
            assert_eq!(c1, ZTP_OK);
        }
        let dx = (biased.displacement_m - clean.displacement_m).abs();
        let expected = bias / (w * w);
        assert!(
            dx > 1e-9 && (dx - expected).abs() / expected < 0.15,
            "well must sit near b/ω_n²: dx {dx} expected {expected}"
        );
    }

    #[test]
    fn coriolis_tether_is_70x_linear() {
        let omega = 0.4;
        let m = 1.0e-7;
        let tether = coriolis_scale_n(m, V_TETHER_M_S, omega);
        let lin = coriolis_scale_n(m, V_LINEAR_M_S, omega);
        assert!((tether / lin - 70.0).abs() < 1e-9);

        let mut a = fresh(0.0);
        let mut b = fresh(0.0);
        let (_, t_lin) = step_tesseract(
            &mut a, 0.0, omega, 0.0, 0.0, 0.0, 0.0, 0.0, V_LINEAR_M_S, 0.001,
        );
        let (_, t_teth) = step_tesseract(
            &mut b, 0.0, omega, 0.0, 0.0, 0.0, 0.0, 0.0, V_TETHER_M_S, 0.001,
        );
        assert!(t_lin.scale_factor_n > 0.0);
        assert!((t_teth.scale_factor_n / t_lin.scale_factor_n - 70.0).abs() < 1e-9);
    }
}
