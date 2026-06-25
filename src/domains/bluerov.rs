// BlueROV2 (Marine Subsurface & Hydrodynamic Coherence): On-die runtime validation for AUVs/ROVs.
//
// THE OCEAN TRUTH: Does the estimated velocity state reconcile with physical invariants?
//
// THE EMBODIMENT: A micro-class ROV (such as the BlueROV2) operating in dynamic ocean currents
// near high-value offshore structures (wind turbine monopiles, oil platforms). In GPS-denied
// environments, it relies on acoustic positioning or DVL. When DVL bottom-lock is lost
// (e.g. crossing a trench), the state estimator drifts. The autopilot, attempting to correct,
// steers the ROV into structural impact.
//
// ZTP checks the expected hydrodynamic drag, thrust, and buoyancy forces against the raw IMU
// acceleration. If they diverge, a tracking failure is declared, prompting a safety recovery override.

pub const RHO_SEAWATER: f64 = 1025.0; // kg/m^3
pub const GRAVITY: f64 = 9.81;        // m/s^2

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct C_BlueRovState {
    pub position: [f64; 3],          // World coordinates [x, y, z] (positive z = altitude/up)
    pub velocity: [f64; 3],          // World velocity [vx, vy, vz]
    pub pitch_roll_yaw: [f64; 3],    // Euler angles [theta (pitch), phi (roll), psi (yaw)]
    pub thruster_commands: [f64; 6], // Normalized inputs [-1.0 to 1.0]. T1-T4 horizontal, T5-T6 vertical
    pub current_velocity: [f64; 3],  // World ocean current velocity [cx, cy, cz]
    pub mass: f64,                   // Dry mass in kg (~11.0 kg)
    pub volume: f64,                 // Displacement volume in m^3 (~0.011 m^3)
    pub drag_coefficients: [f64; 3], // Drag coefficients * Area per axis in body coordinates [Cd*Ax, Cd*Ay, Cd*Az]
    pub max_thrust_horizontal: f64,  // Newtons capacity per horizontal thruster
    pub max_thrust_vertical: f64,    // Newtons capacity per vertical thruster
    pub tether_anchor: [f64; 3],     // Anchor point [x, y, z] (positive z = up)
    pub tether_length: f64,          // Deployed slack length (m)
    pub tether_k: f64,               // Tether elasticity spring constant (N/m)
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct C_BlueRovResult {
    pub imu_acceleration: [f64; 3],   // Expected IMU reading (excluding gravity)
    pub true_acceleration: [f64; 3],  // Net world acceleration (including gravity/buoyancy)
    pub coherence_residual: f64,      // Discrepancy metric (g)
    pub coherence_fail: bool,         // Flag indicating failure
}

impl C_BlueRovState {
    /// Compute the 3D thrust vector in body coordinates using the 45-degree vectored thruster matrix.
    pub fn thrust_vector_body(&self) -> [f64; 3] {
        // T1-T4 are horizontal vectored thrusters at 45 degrees
        // T1: Front-Right (+45 deg)
        // T2: Front-Left (-45 deg)
        // T3: Rear-Right (-135 deg)
        // T4: Rear-Left (+135 deg)
        //
        // Surge (Forward): (T1 + T2 + T3 + T4) * cos(45 deg)
        // Sway (Lateral):  (T1 - T2 - T3 + T4) * sin(45 deg)
        let surge = (self.thruster_commands[0] + self.thruster_commands[1] + 
                     self.thruster_commands[2] + self.thruster_commands[3]) * 0.70710678118 * self.max_thrust_horizontal;
        
        let sway = (self.thruster_commands[0] - self.thruster_commands[1] - 
                    self.thruster_commands[2] + self.thruster_commands[3]) * 0.70710678118 * self.max_thrust_horizontal;
        
        // T5-T6 are vertical thrusters (Heave)
        let heave = (self.thruster_commands[4] + self.thruster_commands[5]) * self.max_thrust_vertical;
        
        [surge, sway, heave]
    }

    /// Rotate a 3D vector from Body frame to World frame using ZYX Euler angles.
    pub fn body_to_world(&self, v_body: [f64; 3]) -> [f64; 3] {
        let theta = self.pitch_roll_yaw[0]; // Pitch
        let phi = self.pitch_roll_yaw[1];   // Roll
        let psi = self.pitch_roll_yaw[2];   // Yaw

        let c_theta = theta.cos();
        let s_theta = theta.sin();
        let c_phi = phi.cos();
        let s_phi = phi.sin();
        let c_psi = psi.cos();
        let s_psi = psi.sin();

        let r00 = c_psi * c_theta;
        let r01 = c_psi * s_theta * s_phi - s_psi * c_phi;
        let r02 = c_psi * s_theta * c_phi + s_psi * s_phi;

        let r10 = s_psi * c_theta;
        let r11 = s_psi * s_theta * s_phi + c_psi * c_phi;
        let r12 = s_psi * s_theta * c_phi - c_psi * s_phi;

        let r20 = -s_theta;
        let r21 = c_theta * s_phi;
        let r22 = c_theta * c_phi;

        [
            r00 * v_body[0] + r01 * v_body[1] + r02 * v_body[2],
            r10 * v_body[0] + r11 * v_body[1] + r12 * v_body[2],
            r20 * v_body[0] + r21 * v_body[1] + r22 * v_body[2],
        ]
    }

    /// Rotate a 3D vector from World frame to Body frame.
    pub fn world_to_body(&self, v_world: [f64; 3]) -> [f64; 3] {
        let theta = self.pitch_roll_yaw[0];
        let phi = self.pitch_roll_yaw[1];
        let psi = self.pitch_roll_yaw[2];

        let c_theta = theta.cos();
        let s_theta = theta.sin();
        let c_phi = phi.cos();
        let s_phi = phi.sin();
        let c_psi = psi.cos();
        let s_psi = psi.sin();

        // Transpose of Body-to-World rotation matrix R (since R is orthogonal, R^T = R^-1)
        let r00 = c_psi * c_theta;
        let r10 = c_psi * s_theta * s_phi - s_psi * c_phi;
        let r20 = c_psi * s_theta * c_phi + s_psi * s_phi;

        let r01 = s_psi * c_theta;
        let r11 = s_psi * s_theta * s_phi + c_psi * c_phi;
        let r21 = s_psi * s_theta * c_phi - c_psi * s_phi;

        let r02 = -s_theta;
        let r12 = c_theta * s_phi;
        let r22 = c_theta * c_phi;

        [
            r00 * v_world[0] + r01 * v_world[1] + r02 * v_world[2],
            r10 * v_world[0] + r11 * v_world[1] + r12 * v_world[2],
            r20 * v_world[0] + r21 * v_world[1] + r22 * v_world[2],
        ]
    }
}

/// Run one 1000Hz step of BlueROV2 dynamics and verify sensor coherence.
pub fn step_bluerov_dynamics(
    state: &mut C_BlueRovState,
    nav_vel: [f64; 3], // Velocity claimed by navigation filter (World frame)
    coherence_threshold: f64,
    dt: f64,
) -> C_BlueRovResult {
    if state.mass <= 0.0 { state.mass = 11.0; }
    if state.volume <= 0.0 { state.volume = 0.011; }
    if dt <= 0.0 {
        return C_BlueRovResult {
            imu_acceleration: [0.0; 3],
            true_acceleration: [0.0; 3],
            coherence_residual: 0.0,
            coherence_fail: false,
        };
    }

    // 1. Compute relative velocity in World frame
    let v_rel_world = [
        state.velocity[0] - state.current_velocity[0],
        state.velocity[1] - state.current_velocity[1],
        state.velocity[2] - state.current_velocity[2],
    ];

    // 2. Rotate relative velocity to Body frame to compute drag
    let v_rel_body = state.world_to_body(v_rel_world);

    // 3. Compute quadratic hydrodynamic drag forces in Body frame
    // F_drag = -0.5 * rho * Cd * Area * |v| * v
    let f_drag_body = [
        -0.5 * RHO_SEAWATER * state.drag_coefficients[0] * v_rel_body[0] * v_rel_body[0].abs(),
        -0.5 * RHO_SEAWATER * state.drag_coefficients[1] * v_rel_body[1] * v_rel_body[1].abs(),
        -0.5 * RHO_SEAWATER * state.drag_coefficients[2] * v_rel_body[2] * v_rel_body[2].abs(),
    ];

    // 4. Compute thrust vector in Body frame
    let f_thrust_body = state.thrust_vector_body();

    // 5. Total contact force in Body frame (thrust + drag)
    let f_contact_body = [
        f_thrust_body[0] + f_drag_body[0],
        f_thrust_body[1] + f_drag_body[1],
        f_thrust_body[2] + f_drag_body[2],
    ];

    // 6. Rotate contact forces to World frame
    let f_contact_world = state.body_to_world(f_contact_body);

    // 7. Gravity and Buoyancy forces (World frame)
    let f_gravity_world = [0.0, 0.0, -state.mass * GRAVITY];
    let f_buoyancy_world = [0.0, 0.0, RHO_SEAWATER * GRAVITY * state.volume];

    // 7.5. Tether Dynamics (Taut spring force and current-induced cable drag)
    let mut f_tether_world = [0.0, 0.0, 0.0];
    if state.tether_k > 0.0 {
        let dx_t = state.tether_anchor[0] - state.position[0];
        let dy_t = state.tether_anchor[1] - state.position[1];
        let dz_t = state.tether_anchor[2] - state.position[2];
        let dist_t = (dx_t * dx_t + dy_t * dy_t + dz_t * dz_t).sqrt();
        
        if dist_t > state.tether_length && dist_t > 0.0 {
            // Taut tether tension pulling ROV toward anchor
            let tension = state.tether_k * (dist_t - state.tether_length);
            let unit_x = dx_t / dist_t;
            let unit_y = dy_t / dist_t;
            let unit_z = dz_t / dist_t;
            f_tether_world[0] += tension * unit_x;
            f_tether_world[1] += tension * unit_y;
            f_tether_world[2] += tension * unit_z;
        }
        
        // Add current-induced tether drag (cylinder cross-section drag)
        // Deployed length is approximately dist_t.
        // Tether relative velocity to water (assuming tether speed ≈ ROV speed / 2 on average)
        let v_tether_mid = [
            state.velocity[0] * 0.5 - state.current_velocity[0],
            state.velocity[1] * 0.5 - state.current_velocity[1],
            state.velocity[2] * 0.5 - state.current_velocity[2],
        ];
        let speed_t_rel = (v_tether_mid[0] * v_tether_mid[0] + v_tether_mid[1] * v_tether_mid[1] + v_tether_mid[2] * v_tether_mid[2]).sqrt();
        if speed_t_rel > 1e-6 {
            let cd_tether = 1.2; // drag of cylinder
            let dia_tether = 0.008; // 8mm diameter cable
            // Total drag on the cable length
            let drag_t_mag = 0.5 * RHO_SEAWATER * speed_t_rel * speed_t_rel * cd_tether * (dia_tether * dist_t);
            // Half of the drag forces are transmitted to the ROV
            f_tether_world[0] -= 0.5 * drag_t_mag * (v_tether_mid[0] / speed_t_rel);
            f_tether_world[1] -= 0.5 * drag_t_mag * (v_tether_mid[1] / speed_t_rel);
            f_tether_world[2] -= 0.5 * drag_t_mag * (v_tether_mid[2] / speed_t_rel);
        }
    }

    // 8. Net world acceleration
    let true_acc = [
        (f_contact_world[0] + f_tether_world[0]) / state.mass,
        (f_contact_world[1] + f_tether_world[1]) / state.mass,
        (f_contact_world[2] + f_gravity_world[2] + f_buoyancy_world[2] + f_tether_world[2]) / state.mass,
    ];

    // 9. IMU proper acceleration (excluding gravity vector)
    // imu_acc = (f_contact + f_buoyancy + f_tether) / mass
    let imu_acc = [
        (f_contact_world[0] + f_tether_world[0]) / state.mass,
        (f_contact_world[1] + f_tether_world[1]) / state.mass,
        (f_contact_world[2] + f_buoyancy_world[2] + f_tether_world[2]) / state.mass,
    ];

    // 10. Euler-Maruyama Integration
    state.velocity[0] += true_acc[0] * dt;
    state.velocity[1] += true_acc[1] * dt;
    state.velocity[2] += true_acc[2] * dt;

    state.position[0] += state.velocity[0] * dt;
    state.position[1] += state.velocity[1] * dt;
    state.position[2] += state.velocity[2] * dt;

    // 11. Coherence Auditor
    // We compute the expected IMU reading based on the navigation filter's reported velocity
    let nav_rel_world = [
        nav_vel[0] - state.current_velocity[0],
        nav_vel[1] - state.current_velocity[1],
        nav_vel[2] - state.current_velocity[2],
    ];
    let nav_rel_body = state.world_to_body(nav_rel_world);

    let f_drag_nav_body = [
        -0.5 * RHO_SEAWATER * state.drag_coefficients[0] * nav_rel_body[0] * nav_rel_body[0].abs(),
        -0.5 * RHO_SEAWATER * state.drag_coefficients[1] * nav_rel_body[1] * nav_rel_body[1].abs(),
        -0.5 * RHO_SEAWATER * state.drag_coefficients[2] * nav_rel_body[2] * nav_rel_body[2].abs(),
    ];

    let f_contact_nav_body = [
        f_thrust_body[0] + f_drag_nav_body[0],
        f_thrust_body[1] + f_drag_nav_body[1],
        f_thrust_body[2] + f_drag_nav_body[2],
    ];

    let f_contact_nav_world = state.body_to_world(f_contact_nav_body);

    let imu_acc_expected = [
        (f_contact_nav_world[0] + f_tether_world[0]) / state.mass,
        (f_contact_nav_world[1] + f_tether_world[1]) / state.mass,
        (f_contact_nav_world[2] + f_buoyancy_world[2] + f_tether_world[2]) / state.mass,
    ];

    // Residual = Euclidean norm of IMU difference (in g units)
    let diff = [
        imu_acc[0] - imu_acc_expected[0],
        imu_acc[1] - imu_acc_expected[1],
        imu_acc[2] - imu_acc_expected[2],
    ];
    let residual_mps2 = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]).sqrt();
    let residual_g = residual_mps2 / GRAVITY;
    let fail = residual_g > coherence_threshold;

    C_BlueRovResult {
        imu_acceleration: imu_acc,
        true_acceleration: true_acc,
        coherence_residual: residual_g,
        coherence_fail: fail,
    }
}
