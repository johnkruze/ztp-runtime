//! Tokamak MHD quench — same vibration as genesis_core `physics/tokamak.rs`.
//! Clock is microseconds. `ztp_tokamak_step(dt_us)`. Not the grasp 1 kHz loop.
//! ProofChain lives on Forge; the Reflex step is the bottle.

pub const MU_0: f64 = 1.25663706e-6;
pub const K_B: f64 = 1.380649e-23;
pub const CONTAINMENT_RADIUS: f64 = 2.0;
pub const PLASMA_MASS_DENSITY: f64 = 2e-8;
pub const DIVERTOR_Z_LIMIT: f64 = 0.5;

pub const L_COIL: f64 = 0.01;
pub const R_0: f64 = 1e-3;
pub const ALPHA_R: f64 = 0.1;
pub const T_CRYO: f64 = 4.2;
pub const T_QUENCH: f64 = 15.0;
pub const C_COIL: f64 = 0.5;
pub const H_COOL: f64 = 0.1;

pub const GAMMA_Z_SQ: f64 = 1000.0;
pub const K_STABILIZE: f64 = -30.0;
pub const K_DISTURB: f64 = 3000.0;

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct C_TokamakState {
    pub plasma_radius: f64,
    pub radial_velocity: f64,
    pub z_displacement: f64,
    pub z_velocity: f64,
    pub temperature: f64,
    pub particle_density: f64,
    pub b_field: f64,
    pub b_field_asymmetry: f64,
    pub pf_voltage: f64,
    pub pf_current: f64,
    pub coil_temp: f64,
    pub residual: f64,
    pub time_us: u64,
    pub coil_quenched: bool,
    pub quenched: bool,
}

impl Default for C_TokamakState {
    fn default() -> Self {
        Self {
            plasma_radius: 1.8,
            radial_velocity: 0.0,
            z_displacement: 0.0,
            z_velocity: 0.0,
            temperature: 1.5e8,
            particle_density: 1.0e20,
            b_field: 0.721,
            b_field_asymmetry: 0.0,
            pf_voltage: 0.0,
            pf_current: 0.0,
            coil_temp: 4.2,
            residual: 0.0,
            time_us: 0,
            coil_quenched: false,
            quenched: false,
        }
    }
}

impl C_TokamakState {
    pub fn plasma_pressure(&self) -> f64 {
        let volume_ratio = (1.8 / self.plasma_radius).powi(2);
        self.particle_density * volume_ratio * K_B * self.temperature
    }

    pub fn magnetic_pressure(&self) -> f64 {
        (self.b_field * self.b_field) / (2.0 * MU_0)
    }

    /// B = sqrt(2 μ0 P_plasma). Lock this before a confined tick.
    pub fn exact_equilibrium_b_field(&self) -> f64 {
        (2.0 * MU_0 * self.plasma_pressure()).sqrt()
    }

    pub fn apply_agentic_ai_field(&mut self, target_b: f64, radial_noise: f64, z_asymmetry_noise: f64) {
        self.b_field = target_b + radial_noise;
        self.b_field_asymmetry = z_asymmetry_noise;
        self.pf_voltage = -80.0 * self.z_displacement - 12.0 * self.z_velocity + z_asymmetry_noise * 50.0;
    }

    /// One MHD tick. `dt_us` is microseconds (Forge dt = 1).
    pub fn step(&mut self, dt_us: f64) {
        if self.quenched {
            return;
        }
        let dt = dt_us * 1e-6;

        let p_plasma = self.plasma_pressure();
        let p_mag = self.magnetic_pressure();
        let radial_acceleration = (p_plasma - p_mag) / PLASMA_MASS_DENSITY;
        self.radial_velocity += radial_acceleration * dt;
        self.plasma_radius += self.radial_velocity * dt;

        let resistance = if self.coil_quenched {
            1.0
        } else {
            R_0 * (1.0 + ALPHA_R * (self.coil_temp - T_CRYO))
        };
        let di_dt = (self.pf_voltage - self.pf_current * resistance) / L_COIL;
        self.pf_current += di_dt * dt;
        let power_dissipated = self.pf_current * self.pf_current * resistance;
        let dt_coil = (power_dissipated / C_COIL - H_COOL * (self.coil_temp - T_CRYO)) * dt;
        self.coil_temp = (self.coil_temp + dt_coil).max(T_CRYO);
        if self.coil_temp >= T_QUENCH && !self.coil_quenched {
            self.coil_quenched = true;
        }

        let vertical_growth_accel = GAMMA_Z_SQ * self.z_displacement;
        let stabilizing_accel = K_STABILIZE * self.pf_current;
        let disturbing_accel = K_DISTURB * self.b_field_asymmetry;
        let z_accel = vertical_growth_accel + stabilizing_accel + disturbing_accel;
        self.z_velocity += z_accel * dt;
        self.z_displacement += self.z_velocity * dt;

        self.time_us = self.time_us.saturating_add(dt_us.round() as u64);

        let p_plasma_final = self.plasma_pressure();
        let p_mag_final = self.magnetic_pressure();
        let radial_accel_final = (p_plasma_final - p_mag_final) / PLASMA_MASS_DENSITY;
        self.residual = (radial_accel_final - radial_acceleration).abs();

        if self.plasma_radius >= CONTAINMENT_RADIUS || self.z_displacement.abs() >= DIVERTOR_Z_LIMIT {
            self.quenched = true;
        }
    }
}

pub fn step_tokamak(state: &mut C_TokamakState, dt_us: f64) {
    state.step(dt_us);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn confined_holds_at_equilibrium() {
        let mut t = C_TokamakState::default();
        t.b_field = t.exact_equilibrium_b_field();
        for _ in 0..500 {
            t.step(1.0);
        }
        assert!(!t.quenched);
        assert!(t.plasma_radius < CONTAINMENT_RADIUS);
        assert!(t.z_displacement.abs() < DIVERTOR_Z_LIMIT);
        assert_eq!(t.time_us, 500);
    }

    #[test]
    fn radial_hits_wall() {
        let mut t = C_TokamakState::default();
        t.b_field = t.exact_equilibrium_b_field();
        t.apply_agentic_ai_field(t.b_field, -0.05, 0.0);
        for _ in 0..500 {
            t.step(1.0);
            if t.quenched {
                break;
            }
        }
        assert!(t.quenched);
        assert!(t.plasma_radius >= CONTAINMENT_RADIUS || t.z_displacement.abs() >= DIVERTOR_Z_LIMIT);
    }
}
