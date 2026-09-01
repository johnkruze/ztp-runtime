//! Vehicle hydroplane — Pacejka + freshwater ρ. 1 kHz chassis.
//!
//! Organ: μ / |y| / yaw on a puddle. NOT Boussinesq soil.
//! `ztp_terran_evaluate_contact` is the soil organ. This file is the car.
//!
//! Forge twin: `vehicle_hydroplane_monte_carlo` (`VehicleDynamicsState` 128 B).
//! File pinout is still LastStateFrame64 (64 B). Do not claim .soma.bin is 128 B.

pub const RHO_FRESHWATER: f32 = 997.0;
pub const M_CHASSIS: f32 = 2000.0;
pub const G: f32 = 9.81;
pub const I_YAW: f32 = 3000.0;
pub const A_FRONT: f32 = 1.4;
pub const B_REAR: f32 = 1.4;
pub const TRACK_W: f32 = 1.6;
pub const H_CG: f32 = 0.6;
pub const R_WHEEL: f32 = 0.35;
pub const I_WHEEL: f32 = 2.0;
pub const HYDRO_MU: f32 = 0.25;
pub const CORNER_LOST_M: f32 = 10.0;
pub const WATER_TRANSITION_M: f32 = 0.3;

/// 32 × f32 = 128 B. Matches Forge `VehicleDynamicsState` cache line.
#[repr(C, align(128))]
#[derive(Clone, Copy, Debug)]
pub struct C_VehicleDynamicsState {
    pub timestamp: f32,
    pub chassis_q: [f32; 6],         // x y z roll pitch yaw
    pub chassis_dq: [f32; 6],        // vx vy vz roll_rate pitch_rate yaw_rate
    pub wheel_q: [f32; 4],
    pub wheel_dq: [f32; 4],
    pub wheel_torques: [f32; 4],
    pub pacejka_jacobians: [f32; 4],
    pub normal_forces: [f32; 2],
    pub thermal_accumulated: f32,
}

const _: () = assert!(core::mem::size_of::<C_VehicleDynamicsState>() == 128);

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct C_VehicleHydroplaneResult {
    pub mu: f32,
    pub abs_y_m: f32,
    pub yaw_rad: f32,
    pub hydroplane: bool,
    pub corner_lost: bool,
    pub grip: bool,
}

fn pacejka_lateral_force(alpha: f32, fz: f32, mu: f32) -> f32 {
    let b = 10.0f32;
    let c = 1.3f32;
    let d = mu * fz;
    let e = -1.0f32;
    d * (c * (b * alpha - e * (b * alpha - (b * alpha).atan()))).atan().sin()
}

fn pacejka_longitudinal_force(kappa: f32, fz: f32, mu: f32) -> f32 {
    let b = 12.0f32;
    let c = 1.6f32;
    let d = mu * fz;
    let e = -0.5f32;
    d * (c * (b * kappa - e * (b * kappa - (b * kappa).atan()))).atan().sin()
}

/// One 1 kHz Pacejka chassis step. Host mixes nothing — puddle μ lives here.
/// `ztp_terran_evaluate_contact` is soil; do not call it from this organ.
pub fn step_vehicle_hydroplane(
    state: &mut C_VehicleDynamicsState,
    mu_dry: f32,
    mu_wet: f32,
    x_water_m: f32,
    steer_rad: f32,
    mass_kg: f32,
    v_hold_ms: f32,
    dt: f32,
) -> C_VehicleHydroplaneResult {
    if dt <= 0.0 {
        let y = state.chassis_q[1];
        return C_VehicleHydroplaneResult {
            mu: mu_dry.max(0.0),
            abs_y_m: y.abs(),
            yaw_rad: state.chassis_q[5],
            hydroplane: false,
            corner_lost: y.abs() < CORNER_LOST_M,
            grip: y.abs() >= CORNER_LOST_M,
        };
    }

    let m = if mass_kg > 1.0 { mass_kg } else { M_CHASSIS };
    let i_yaw = I_YAW * (m / M_CHASSIS);

    let mut pos_x = state.chassis_q[0];
    let mut pos_y = state.chassis_q[1];
    let mut roll = state.chassis_q[3];
    let mut pitch = state.chassis_q[4];
    let mut yaw = state.chassis_q[5];

    let mut vel_x = state.chassis_dq[0];
    let mut vel_y = state.chassis_dq[1];
    let mut roll_rate = state.chassis_dq[3];
    let mut pitch_rate = state.chassis_dq[4];
    let mut yaw_rate = state.chassis_dq[5];
    let mut wheel_q = state.wheel_q;
    let mut wheel_dq = state.wheel_dq;

    if vel_x.abs() < 0.1 {
        vel_x = 0.1;
    }

    let terrain_moisture = if pos_x >= x_water_m {
        ((pos_x - x_water_m) / WATER_TRANSITION_M).min(1.0)
    } else {
        0.0
    };
    let rho_scale = RHO_FRESHWATER / 1000.0;
    let mu_wet_dynamic = mu_wet / (1.0 + 0.0005 * vel_x * vel_x * rho_scale);
    let mu_actual = mu_dry - (mu_dry - mu_wet_dynamic) * terrain_moisture;

    let lat_accel = vel_x * yaw_rate;
    let lon_accel = 0.0f32;
    let delta_fz_lat = m * lat_accel * H_CG / TRACK_W;
    let delta_fz_lon = m * lon_accel * H_CG / (A_FRONT + B_REAR);

    let fz_static = 0.25 * m * G;
    let fz_fl = (fz_static - 0.5 * delta_fz_lat + 0.5 * delta_fz_lon).max(100.0);
    let fz_fr = (fz_static + 0.5 * delta_fz_lat + 0.5 * delta_fz_lon).max(100.0);
    let fz_rl = (fz_static - 0.5 * delta_fz_lat - 0.5 * delta_fz_lon).max(100.0);
    let fz_rr = (fz_static + 0.5 * delta_fz_lat - 0.5 * delta_fz_lon).max(100.0);

    let vel_x_fl = vel_x + yaw_rate * (TRACK_W * 0.5);
    let vel_x_fr = vel_x - yaw_rate * (TRACK_W * 0.5);
    let vel_x_rl = vel_x + yaw_rate * (TRACK_W * 0.5);
    let vel_x_rr = vel_x - yaw_rate * (TRACK_W * 0.5);

    let steer_fl = steer_rad + 0.05 * steer_rad * steer_rad.signum();
    let steer_fr = steer_rad - 0.05 * steer_rad * steer_rad.signum();

    let a_fl = steer_fl - ((vel_y + A_FRONT * yaw_rate) / vel_x_fl.max(0.1)).atan();
    let a_fr = steer_fr - ((vel_y + A_FRONT * yaw_rate) / vel_x_fr.max(0.1)).atan();
    let a_rl = -((vel_y - B_REAR * yaw_rate) / vel_x_rl.max(0.1)).atan();
    let a_rr = -((vel_y - B_REAR * yaw_rate) / vel_x_rr.max(0.1)).atan();

    let k_fl = (wheel_dq[0] * R_WHEEL - vel_x_fl) / vel_x_fl.max(0.1);
    let k_fr = (wheel_dq[1] * R_WHEEL - vel_x_fr) / vel_x_fr.max(0.1);
    let k_rl = (wheel_dq[2] * R_WHEEL - vel_x_rl) / vel_x_rl.max(0.1);
    let k_rr = (wheel_dq[3] * R_WHEEL - vel_x_rr) / vel_x_rr.max(0.1);

    let f_y_fl = pacejka_lateral_force(a_fl, fz_fl, mu_actual);
    let f_y_fr = pacejka_lateral_force(a_fr, fz_fr, mu_actual);
    let f_y_rl = pacejka_lateral_force(a_rl, fz_rl, mu_actual);
    let f_y_rr = pacejka_lateral_force(a_rr, fz_rr, mu_actual);

    let f_x_fl = pacejka_longitudinal_force(k_fl, fz_fl, mu_actual);
    let f_x_fr = pacejka_longitudinal_force(k_fr, fz_fr, mu_actual);
    let f_x_rl = pacejka_longitudinal_force(k_rl, fz_rl, mu_actual);
    let f_x_rr = pacejka_longitudinal_force(k_rr, fz_rr, mu_actual);

    let j_fl = (pacejka_longitudinal_force(k_fl + 0.001, fz_fl, mu_actual) - f_x_fl) / 0.001;
    let j_fr = (pacejka_longitudinal_force(k_fr + 0.001, fz_fr, mu_actual) - f_x_fr) / 0.001;
    let j_rl = (pacejka_longitudinal_force(k_rl + 0.001, fz_rl, mu_actual) - f_x_rl) / 0.001;
    let j_rr = (pacejka_longitudinal_force(k_rr + 0.001, fz_rr, mu_actual) - f_x_rr) / 0.001;

    let f_drag = 0.5 * 1.225 * 0.3 * 2.2 * vel_x * vel_x;
    let v_hold = if v_hold_ms > 1.0 { v_hold_ms } else { vel_x.max(1.0) };
    let drive_cmd = (50.0 * (v_hold - vel_x)).clamp(0.0, 800.0);
    let drive_nom = drive_cmd * 0.25;
    let mut drive = [drive_nom; 4];
    let mut brake = [0.0f32; 4];

    let target_spin = 1.5f32;
    let slips = [
        (wheel_dq[0] - vel_x / R_WHEEL).max(0.0),
        (wheel_dq[1] - vel_x / R_WHEEL).max(0.0),
        (wheel_dq[2] - vel_x / R_WHEEL).max(0.0),
        (wheel_dq[3] - vel_x / R_WHEEL).max(0.0),
    ];
    for i in 0..4 {
        if slips[i] > target_spin {
            brake[i] = 300.0;
            drive[i] = 0.0;
        }
    }
    let yaw_target = vel_x * steer_rad / (A_FRONT + B_REAR);
    let yaw_error = yaw_rate - yaw_target;
    if yaw_error.abs() > 0.15 {
        if yaw_error > 0.0 {
            brake[1] += 400.0;
            drive[1] = 0.0;
        } else {
            brake[0] += 400.0;
            drive[0] = 0.0;
        }
    }
    let torques = [
        drive[0] - brake[0],
        drive[1] - brake[1],
        drive[2] - brake[2],
        drive[3] - brake[3],
    ];
    let f_x = [f_x_fl, f_x_fr, f_x_rl, f_x_rr];
    for i in 0..4 {
        wheel_dq[i] += ((torques[i] - f_x[i] * R_WHEEL) / I_WHEEL) * dt;
        wheel_dq[i] = wheel_dq[i].max(0.0);
        wheel_q[i] += wheel_dq[i] * dt;
    }

    let torque_sq = torques[0].powi(2) + torques[1].powi(2) + torques[2].powi(2) + torques[3].powi(2);
    let thermal = state.thermal_accumulated + (torque_sq / 1000.0) * dt;

    let f_x_total = (f_x_fl + f_x_fr) * steer_rad.cos() - (f_y_fl + f_y_fr) * steer_rad.sin()
        + f_x_rl
        + f_x_rr
        - f_drag;
    let f_y_total = (f_x_fl + f_x_fr) * steer_rad.sin() + (f_y_fl + f_y_fr) * steer_rad.cos()
        + f_y_rl
        + f_y_rr;
    let torque_yaw = A_FRONT
        * ((f_x_fl + f_x_fr) * steer_rad.sin() + (f_y_fl + f_y_fr) * steer_rad.cos())
        - B_REAR * (f_y_rl + f_y_rr)
        + (TRACK_W * 0.5)
            * ((f_x_fr - f_x_fl) * steer_rad.cos() - (f_y_fr - f_y_fl) * steer_rad.sin()
                + (f_x_rr - f_x_rl));

    vel_x += (f_x_total / m + vel_y * yaw_rate) * dt;
    vel_y += (f_y_total / m - vel_x * yaw_rate) * dt;
    yaw_rate += (torque_yaw / i_yaw) * dt;
    yaw += yaw_rate * dt;

    pos_x += (vel_x * yaw.cos() - vel_y * yaw.sin()) * dt;
    pos_y += (vel_x * yaw.sin() + vel_y * yaw.cos()) * dt;

    let k_roll = 25000.0f32;
    let d_roll = 1500.0f32;
    let k_pitch = 30000.0f32;
    let d_pitch = 1800.0f32;
    let roll_accel = (f_y_total * H_CG - k_roll * roll - d_roll * roll_rate) / 1000.0;
    let pitch_accel = (f_x_total * H_CG - k_pitch * pitch - d_pitch * pitch_rate) / 1000.0;
    roll_rate += roll_accel * dt;
    roll += roll_rate * dt;
    pitch_rate += pitch_accel * dt;
    pitch += pitch_rate * dt;
    let pos_z = H_CG - 0.25 * (roll.abs() + pitch.abs());

    state.timestamp += dt;
    state.chassis_q = [pos_x, pos_y, pos_z, roll, pitch, yaw];
    state.chassis_dq = [vel_x, vel_y, 0.0, roll_rate, pitch_rate, yaw_rate];
    state.wheel_q = wheel_q;
    state.wheel_dq = wheel_dq;
    state.wheel_torques = torques;
    state.pacejka_jacobians = [j_fl, j_fr, j_rl, j_rr];
    state.normal_forces = [fz_fl + fz_fr, fz_rl + fz_rr];
    state.thermal_accumulated = thermal;

    let abs_y = pos_y.abs();
    let hydro = mu_actual < HYDRO_MU;
    let lost = abs_y < CORNER_LOST_M;
    C_VehicleHydroplaneResult {
        mu: mu_actual,
        abs_y_m: abs_y,
        yaw_rad: yaw,
        hydroplane: hydro,
        corner_lost: lost,
        grip: !hydro && !lost,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cache_line_is_128() {
        assert_eq!(core::mem::size_of::<C_VehicleDynamicsState>(), 128);
    }
}
