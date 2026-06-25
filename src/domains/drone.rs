// Drone (Flight & VSLAM Coherence): On-die runtime validation for tactical UAS.
//
// THE FLIGHT TRUTH: Does the visual sensor track static geometry or moving particulates?
//
// THE EMBODIMENT: An autonomous drone flies in unstructured environments (tactical smoke,
// dust storms, sand). Its visual SLAM (VSLAM) tracks optical flow to estimate velocity.
// If it enters smoke, features on the moving smoke drift dominate the camera, hallucinating
// false velocity reports. The autopilot over-corrects, causing a high-speed crash.
//
// ZTP resolves this: it compares the proper acceleration from the IMU against the first
// derivative of the VSLAM velocity. If the physical IMU reading and the visual velocity derivative
// diverge (meaning the camera claims motion but the accelerometer detects no physical force),
// the residual spikes. The runtime rejects VSLAM, fallback to IMU dead-reckoning occurs,
// and flight stability is preserved.

pub const GRAVITY: f64 = 9.81;

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct C_DroneState {
    pub position: [f64; 3],          // meters [x, y, z]
    pub velocity: [f64; 3],          // m/s [vx, vy, vz]
    pub pitch_roll_yaw: [f64; 3],    // radians [theta, phi, psi]
    pub motor_rpm: [f64; 4],         // normalized throttle [0.0 - 1.0] for 4 rotors
    pub wind_velocity: [f64; 3],     // m/s [wx, wy, wz]
    pub mass: f64,                   // kg
    pub drag_coefficient: f64,      // N/(m/s) linear drag
    pub max_thrust: f64,             // Newtons max thrust capacity
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct C_DroneResult {
    pub imu_acceleration: [f64; 3],   // Proper acceleration read by IMU [ax, ay, az]
    pub true_acceleration: [f64; 3],  // Net physical acceleration in world frame
    pub coherence_residual: f64,       // Mismatch metric
    pub coherence_fail: bool,         // Flag indicating VSLAM tracking failure
}

impl C_DroneState {
    /// Compute physical thrust vector in world coordinates from rotor commands
    pub fn thrust_vector(&self) -> [f64; 3] {
        // Average motor throttle
        let throttle = (self.motor_rpm[0] + self.motor_rpm[1] + self.motor_rpm[2] + self.motor_rpm[3]) / 4.0;
        let t_mag = throttle * self.max_thrust;

        let theta = self.pitch_roll_yaw[0]; // Pitch
        let phi = self.pitch_roll_yaw[1];   // Roll
        let psi = self.pitch_roll_yaw[2];   // Yaw

        // ZYX rotation matrix representation of body Z-axis (thrust vector)
        let rx = theta.sin() * psi.cos() * phi.cos() + phi.sin() * psi.sin();
        let ry = theta.sin() * psi.sin() * phi.cos() - phi.sin() * psi.cos();
        let rz = theta.cos() * phi.cos();

        [t_mag * rx, t_mag * ry, t_mag * rz]
    }

    /// Compute air drag forces based on true airspeed
    pub fn drag_vector(&self) -> [f64; 3] {
        let v_rel = [
            self.velocity[0] - self.wind_velocity[0],
            self.velocity[1] - self.wind_velocity[1],
            self.velocity[2] - self.wind_velocity[2],
        ];
        [
            -self.drag_coefficient * v_rel[0],
            -self.drag_coefficient * v_rel[1],
            -self.drag_coefficient * v_rel[2],
        ]
    }
}

/// Run one 1000Hz integration step of drone physics
pub fn step_drone_dynamics(
    state: &mut C_DroneState,
    vslam_vel: [f64; 3],
    vslam_vel_prev: [f64; 3],
    dt: f64,
    coherence_threshold: f64,
) -> C_DroneResult {
    if state.mass <= 0.0 { state.mass = 1.5; }
    if dt <= 0.0 { return C_DroneResult { imu_acceleration: [0.0; 3], true_acceleration: [0.0; 3], coherence_residual: 0.0, coherence_fail: false }; }

    // 1. Calculate forces
    let f_thrust = state.thrust_vector();
    let f_drag = state.drag_vector();

    // 2. Net acceleration (forces / mass - gravity vector)
    let true_acc = [
        (f_thrust[0] + f_drag[0]) / state.mass,
        (f_thrust[1] + f_drag[1]) / state.mass,
        (f_thrust[2] + f_drag[2]) / state.mass - GRAVITY,
    ];

    // 3. IMU proper acceleration (excluding gravity vector)
    let imu_acc = [
        (f_thrust[0] + f_drag[0]) / state.mass,
        (f_thrust[1] + f_drag[1]) / state.mass,
        (f_thrust[2] + f_drag[2]) / state.mass,
    ];

    // 4. Update physical state (Euler-Maruyama integration)
    state.velocity[0] += true_acc[0] * dt;
    state.velocity[1] += true_acc[1] * dt;
    state.velocity[2] += true_acc[2] * dt;

    state.position[0] += state.velocity[0] * dt;
    state.position[1] += state.velocity[1] * dt;
    state.position[2] += state.velocity[2] * dt;

    // 5. Coherence Audit
    // Estimate acceleration from visual SLAM velocity derivative
    let a_vslam = [
        (vslam_vel[0] - vslam_vel_prev[0]) / dt,
        (vslam_vel[1] - vslam_vel_prev[1]) / dt,
        (vslam_vel[2] - vslam_vel_prev[2]) / dt,
    ];

    // Expected IMU readings based on visual velocity change:
    // proper acceleration = true acceleration + gravity
    let imu_exp = [
        a_vslam[0],
        a_vslam[1],
        a_vslam[2] + GRAVITY,
    ];

    // Residual = Euclidean norm of IMU difference
    let diff = [
        imu_acc[0] - imu_exp[0],
        imu_acc[1] - imu_exp[1],
        imu_acc[2] - imu_exp[2],
    ];
    let residual = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]).sqrt();
    let fail = residual > coherence_threshold;

    C_DroneResult {
        imu_acceleration: imu_acc,
        true_acceleration: true_acc,
        coherence_residual: residual,
        coherence_fail: fail,
    }
}
