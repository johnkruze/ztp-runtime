//! Swing equation — same vibration as genesis_core `physics/swing.rs`.
//! Clock is 1 ms. `ztp_swing_step(dt)`. Numeric 1 kHz is this rotor, not the grasp loop.

pub const NOMINAL_FREQUENCY: f64 = 60.0;
pub const OMEGA_S: f64 = 2.0 * std::f64::consts::PI * NOMINAL_FREQUENCY;
pub const PI: f64 = std::f64::consts::PI;
pub const K_PLL: f64 = 20.0;
pub const PLL_TRIP_LIMIT: f64 = 0.35;
pub const GOVERNOR_TC: f64 = 0.5;
pub const GOVERNOR_DROOP: f64 = 10.0;

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct C_SwingState {
    pub h_constant: f64,
    pub damping: f64,
    pub p_mech: f64,
    pub p_max: f64,
    pub delta: f64,
    pub delta_omega: f64,
    pub inverter_fraction: f64,
    pub pll_err: f64,
    pub governor_valve: f64,
    pub residual: f64,
    pub time_ms: u64,
    pub inverter_tripped: bool,
    pub cascaded: bool,
}

impl Default for C_SwingState {
    fn default() -> Self {
        let p_mech: f64 = 1.0;
        let p_max: f64 = 1.04;
        let initial_delta = (p_mech / p_max).asin();
        Self {
            h_constant: 5.0,
            damping: 0.05,
            p_mech,
            p_max,
            delta: initial_delta,
            delta_omega: 0.0,
            inverter_fraction: 0.30,
            pll_err: 0.0,
            governor_valve: 1.0,
            residual: 0.0,
            time_ms: 0,
            inverter_tripped: false,
            cascaded: false,
        }
    }
}

impl C_SwingState {
    pub fn p_elec(&self) -> f64 {
        self.p_max * self.delta.sin()
    }

    pub fn effective_inertia(&self) -> f64 {
        if self.inverter_tripped {
            self.h_constant
        } else {
            self.h_constant * (1.0 - self.inverter_fraction)
        }
    }

    pub fn ai_apply_load_mismatch(&mut self, mismatch_percentage: f64) {
        let reduction = 1.0 - (mismatch_percentage / 100.0);
        self.p_max *= reduction;
    }

    pub fn simulate_weather_loss(&mut self, loss_percentage: f64) {
        let reduction = 1.0 - (loss_percentage / 100.0);
        self.p_mech *= reduction;
        if self.inverter_fraction > 0.0 && !self.inverter_tripped {
            self.p_max *= 1.0 - (self.inverter_fraction * loss_percentage / 100.0);
        }
    }

    /// One swing tick. `dt` is seconds (Forge dt = 0.001).
    pub fn step(&mut self, dt: f64) {
        if self.cascaded {
            return;
        }

        if !self.inverter_tripped && self.inverter_fraction > 0.0 {
            let d_pll_err = self.delta_omega - K_PLL * self.pll_err;
            self.pll_err += d_pll_err * dt;
            if self.pll_err.abs() >= PLL_TRIP_LIMIT {
                self.inverter_tripped = true;
                self.p_max *= 1.0 - self.inverter_fraction;
            }
        }

        let f_deviation = self.delta_omega / (2.0 * PI);
        let target_valve = (1.0 - GOVERNOR_DROOP * (f_deviation / NOMINAL_FREQUENCY)).clamp(0.5, 1.5);
        let dg = (target_valve - self.governor_valve) / GOVERNOR_TC;
        self.governor_valve += dg * dt;
        let current_p_mech = self.p_mech * self.governor_valve;

        let p_e = self.p_elec();
        let h_eff = self.effective_inertia().max(0.1);
        let acceleration = (PI * NOMINAL_FREQUENCY / h_eff)
            * (current_p_mech - p_e - self.damping * self.delta_omega);

        self.delta_omega += acceleration * dt;
        self.delta += self.delta_omega * dt;
        self.time_ms = self.time_ms.saturating_add((dt * 1000.0).round() as u64);

        let p_e_final = self.p_elec();
        let imbalance = current_p_mech * self.governor_valve - p_e_final - self.damping * self.delta_omega;
        self.residual = imbalance.abs();

        if self.delta >= core::f64::consts::FRAC_PI_2 {
            self.cascaded = true;
        }
    }
}

pub fn step_swing(state: &mut C_SwingState, dt: f64) {
    state.step(dt);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn held_at_nominal() {
        let mut s = C_SwingState::default();
        for _ in 0..200 {
            s.step(0.001);
        }
        assert!(!s.cascaded);
        assert!(s.delta < core::f64::consts::FRAC_PI_2);
        assert_eq!(s.time_ms, 200);
    }

    #[test]
    fn cascade_on_shock() {
        let mut s = C_SwingState::default();
        s.h_constant = 1.0;
        s.p_mech = 1.0;
        s.p_max = 1.05;
        s.inverter_fraction = 0.0;
        s.delta = (s.p_mech / s.p_max).asin();
        s.ai_apply_load_mismatch(40.0);
        for _ in 0..400 {
            s.step(0.001);
            if s.cascaded {
                break;
            }
        }
        assert!(s.cascaded);
    }
}
