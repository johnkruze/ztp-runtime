pub mod domains;

// ─── DETERMINISTIC LCG PRNG ─────────────────────────────────────
pub mod rng {
    pub struct Rng(u64);

    impl Rng {
        pub fn new(seed: u64) -> Self {
            Self(seed)
        }

        pub fn next_u64(&mut self) -> u64 {
            self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            self.0
        }

        pub fn next_f64(&mut self) -> f64 {
            (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
        }

        pub fn range(&mut self, lo: f64, hi: f64) -> f64 {
            lo + self.next_f64() * (hi - lo)
        }

        pub fn chance(&mut self, probability: f64) -> bool {
            self.next_f64() < probability
        }

        pub fn index(&mut self, n: usize) -> usize {
            if n == 0 { return 0; }
            (self.next_f64() * n as f64) as usize % n
        }

        pub fn gaussian(&mut self, mean: f64, std_dev: f64) -> f64 {
            let u1 = self.next_f64().max(1e-15);
            let u2 = self.next_f64();
            let z = (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos();
            mean + z * std_dev
        }
    }
}

// ─── CUSTOM ZERO-DEPENDENCY CRYPTO (SHA-256 & HEX) ──────────────
pub mod crypto {
    #[derive(Clone)]
    pub struct Sha256 {
        state: [u32; 8],
        buffer: [u8; 64],
        len: u64,
    }

    impl Sha256 {
        pub fn new() -> Self {
            Self {
                state: [
                    0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                    0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19,
                ],
                buffer: [0; 64],
                len: 0,
            }
        }

        pub fn update(&mut self, data: &[u8]) {
            let mut idx = (self.len % 64) as usize;
            self.len += data.len() as u64;
            let mut data_idx = 0;
            while data_idx < data.len() {
                let space = 64 - idx;
                let chunk_len = space.min(data.len() - data_idx);
                self.buffer[idx..idx + chunk_len].copy_from_slice(&data[data_idx..data_idx + chunk_len]);
                idx += chunk_len;
                data_idx += chunk_len;
                if idx == 64 {
                    let buf = self.buffer;
                    self.compress(&buf);
                    idx = 0;
                }
            }
        }

        pub fn finalize(mut self) -> [u8; 32] {
            let bit_len = self.len * 8;
            self.update(&[0x80]);
            let buffer_len = (self.len % 64) as usize;
            if buffer_len > 56 {
                let zeros = [0u8; 64];
                self.update(&zeros[..64 - buffer_len]);
            }
            let buffer_len = (self.len % 64) as usize;
            let pad_len = 56 - buffer_len;
            let zeros = [0u8; 64];
            self.update(&zeros[..pad_len]);
            let len_bytes = bit_len.to_be_bytes();
            self.update(&len_bytes);
            
            let mut out = [0u8; 32];
            for i in 0..8 {
                out[i*4..(i+1)*4].copy_from_slice(&self.state[i].to_be_bytes());
            }
            out
        }

        fn compress(&mut self, block: &[u8; 64]) {
            const K: [u32; 64] = [
                0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
                0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
                0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
                0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
                0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
                0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
                0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
                0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
            ];

            let mut w = [0u32; 64];
            for i in 0..16 {
                w[i] = u32::from_be_bytes([
                    block[i*4], block[i*4+1], block[i*4+2], block[i*4+3]
                ]);
            }
            for i in 16..64 {
                let s0 = w[i-15].rotate_right(7) ^ w[i-15].rotate_right(18) ^ (w[i-15] >> 3);
                let s1 = w[i-2].rotate_right(17) ^ w[i-2].rotate_right(19) ^ (w[i-2] >> 10);
                w[i] = w[i-16].wrapping_add(s0).wrapping_add(w[i-7]).wrapping_add(s1);
            }

            let mut a = self.state[0];
            let mut b = self.state[1];
            let mut c = self.state[2];
            let mut d = self.state[3];
            let mut e = self.state[4];
            let mut f = self.state[5];
            let mut g = self.state[6];
            let mut h = self.state[7];

            for i in 0..64 {
                let s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
                let ch = (e & f) ^ ((!e) & g);
                let temp1 = h.wrapping_add(s1).wrapping_add(ch).wrapping_add(K[i]).wrapping_add(w[i]);
                let s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
                let maj = (a & b) ^ (a & c) ^ (b & c);
                let temp2 = s0.wrapping_add(maj);

                h = g;
                g = f;
                f = e;
                e = d.wrapping_add(temp1);
                d = c;
                c = b;
                b = a;
                a = temp1.wrapping_add(temp2);
            }

            self.state[0] = self.state[0].wrapping_add(a);
            self.state[1] = self.state[1].wrapping_add(b);
            self.state[2] = self.state[2].wrapping_add(c);
            self.state[3] = self.state[3].wrapping_add(d);
            self.state[4] = self.state[4].wrapping_add(e);
            self.state[5] = self.state[5].wrapping_add(f);
            self.state[6] = self.state[6].wrapping_add(g);
            self.state[7] = self.state[7].wrapping_add(h);
        }
    }

    pub fn hex_encode(bytes: &[u8]) -> String {
        let mut s = String::with_capacity(bytes.len() * 2);
        for &b in bytes {
            s.push(char::from_digit((b >> 4) as u32, 16).unwrap());
            s.push(char::from_digit((b & 0xf) as u32, 16).unwrap());
        }
        s
    }

    #[derive(Clone)]
    pub struct ProofChain {
        hasher: Sha256,
        feeds: u64,
    }

    impl ProofChain {
        pub fn new() -> Self {
            Self {
                hasher: Sha256::new(),
                feeds: 0,
            }
        }

        pub fn seed(&mut self, data: &[u8]) {
            self.hasher.update(data);
        }

        pub fn feed(&mut self, data: &[u8]) {
            self.hasher.update(data);
            self.feeds += 1;
        }

        pub fn feed_f64(&mut self, val: f64) {
            self.hasher.update(&val.to_le_bytes());
        }

        pub fn feed_str(&mut self, s: &str) {
            self.hasher.update(s.as_bytes());
        }

        pub fn feed_count(&self) -> u64 {
            self.feeds
        }

        pub fn seal(self) -> String {
            hex_encode(&self.hasher.finalize())
        }
    }

    pub fn seal_run(proof_hashes: &[String]) -> String {
        let mut hasher = Sha256::new();
        for h in proof_hashes {
            hasher.update(h.as_bytes());
        }
        hex_encode(&hasher.finalize())
    }
}

// ─── C-COMPATIBLE FFI LAYER ─────────────────────────────────────
use crate::domains::terran::{SoilType, Locomotion};

#[repr(C)]
pub struct C_SoilResult {
    pub max_compaction: f64,
    pub compaction_depth_m: f64,
}

#[no_mangle]
pub extern "C" fn ztp_terran_evaluate_contact(
    soil_type_code: i32,
    moisture: f64,
    glomalin_mg_g: f64,
    compaction: f64,
    depth_layers: u32,
    mass_kg: f64,
    footprint_m2: f64,
    locomotion_code: i32,
) -> C_SoilResult {
    let soil_type = match soil_type_code {
        0 => SoilType::Sand,
        1 => SoilType::Loam,
        2 => SoilType::Clay,
        _ => SoilType::Andisol,
    };
    let locomotion = match locomotion_code {
        0 => Locomotion::Wheeled,
        1 => Locomotion::Tracked,
        2 => Locomotion::Legged,
        _ => Locomotion::Drone,
    };

    let profile = domains::terran::SoilProfile {
        soil_type,
        moisture,
        glomalin_mg_g,
        compaction,
        depth_layers: depth_layers as usize,
    };

    let robot = domains::terran::RobotContact {
        mass_kg,
        footprint_m2,
        locomotion,
    };

    let (max_c, depth) = profile.evaluate_contact(&robot);
    C_SoilResult {
        max_compaction: max_c,
        compaction_depth_m: depth,
    }
}

#[repr(C)]
pub struct C_SatelliteState {
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub quaternion_attitude: [f64; 4],
    pub angular_velocity: [f64; 3],
    pub inertia_tensor: [f64; 9],
}

#[no_mangle]
pub extern "C" fn ztp_orbital_step_6dof(
    state: *mut C_SatelliteState,
    dt: f64,
) {
    if state.is_null() { return; }
    unsafe {
        let s = &mut *state;
        let physics = domains::orbital::OrbitalPhysics::default();
        
        // Map from C struct to internal representation
        let mut rust_state = domains::orbital::SatelliteState {
            position: s.position,
            velocity: s.velocity,
            quaternion_attitude: s.quaternion_attitude,
            angular_velocity: s.angular_velocity,
            inertia_tensor: [
                [s.inertia_tensor[0], s.inertia_tensor[1], s.inertia_tensor[2]],
                [s.inertia_tensor[3], s.inertia_tensor[4], s.inertia_tensor[5]],
                [s.inertia_tensor[6], s.inertia_tensor[7], s.inertia_tensor[8]],
            ],
        };

        physics.step_6dof(&mut rust_state, dt);

        s.position = rust_state.position;
        s.velocity = rust_state.velocity;
    }
}

#[no_mangle]
pub extern "C" fn ztp_orbital_step_attitude(
    state: *mut C_SatelliteState,
    ext_torque_x: f64,
    ext_torque_y: f64,
    ext_torque_z: f64,
    dt: f64,
) {
    if state.is_null() { return; }
    unsafe {
        let s = &mut *state;
        
        let mut rust_state = domains::orbital::SatelliteState {
            position: s.position,
            velocity: s.velocity,
            quaternion_attitude: s.quaternion_attitude,
            angular_velocity: s.angular_velocity,
            inertia_tensor: [
                [s.inertia_tensor[0], s.inertia_tensor[1], s.inertia_tensor[2]],
                [s.inertia_tensor[3], s.inertia_tensor[4], s.inertia_tensor[5]],
                [s.inertia_tensor[6], s.inertia_tensor[7], s.inertia_tensor[8]],
            ],
        };

        domains::orbital::OrbitalPhysics::step_attitude(
            &mut rust_state,
            &[ext_torque_x, ext_torque_y, ext_torque_z],
            dt,
        );

        s.quaternion_attitude = rust_state.quaternion_attitude;
        s.angular_velocity = rust_state.angular_velocity;
    }
}

#[repr(C)]
pub struct C_HandshakeResult {
    pub success: bool,
    pub resonance: f64,
    pub avg_snr_db: f64,
}

#[no_mangle]
pub extern "C" fn ztp_atheric_handshake(
    seed_bytes: *const u8,
    strength: f64,
    distance_km: f64,
) -> C_HandshakeResult {
    if seed_bytes.is_null() {
        return C_HandshakeResult { success: false, resonance: 0.0, avg_snr_db: -200.0 };
    }
    unsafe {
        let mut seed = [0u8; 32];
        std::ptr::copy_nonoverlapping(seed_bytes, seed.as_mut_ptr(), 32);
        
        let mut sys = domains::atheric::AthericSystem::new(8, strength, -120.0, distance_km);
        sys.hop_seed = seed;
        let snr = sys.avg_snr_db();
        let coherence = sys.coherence(3.0);
        
        C_HandshakeResult {
            success: coherence > domains::atheric::RESONANCE_GATE,
            resonance: coherence,
            avg_snr_db: snr,
        }
    }
}

#[repr(C)]
pub struct C_MarsState {
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub dry_mass: f64,
    pub drag_area: f64,
    pub cd: f64,
    pub fuel_mass: f64,
    pub specific_impulse: f64,
}

#[repr(C)]
pub struct C_MarsResult {
    pub density: f64,
    pub drag_force: [f64; 3],
    pub net_accel: [f64; 3],
}

#[no_mangle]
pub extern "C" fn ztp_mars_step(
    state: *mut C_MarsState,
    retro_thrust: f64,
    dt: f64,
) -> C_MarsResult {
    if state.is_null() {
        return C_MarsResult {
            density: 0.0,
            drag_force: [0.0; 3],
            net_accel: [0.0; 3],
        };
    }
    unsafe {
        let s = &mut *state;
        let mut vehicle = domains::mars::EdlVehicle {
            dry_mass: s.dry_mass,
            drag_area: s.drag_area,
            cd: s.cd,
            fuel_mass: s.fuel_mass,
            specific_impulse: s.specific_impulse,
        };

        let (density, drag, accel) = vehicle.step(
            &mut s.position,
            &mut s.velocity,
            retro_thrust,
            dt,
        );

        s.fuel_mass = vehicle.fuel_mass;

        C_MarsResult {
            density,
            drag_force: drag,
            net_accel: accel,
        }
    }
}

// Expose Dexterous Tactile Grasp FFI wrappers
pub use crate::domains::dexterous::{
    C_TactileArray, C_GraspState, C_GraspResult, C_SurgicalTissueAuditor,
    C_SurgicalResult, C_MicroReleaseAuditor, C_MicroResult,
};

#[no_mangle]
pub extern "C" fn ztp_dexterous_evaluate_grasp(
    sensor_data: *const C_TactileArray,
    state: *mut C_GraspState,
    dt: f32,
) -> C_GraspResult {
    if sensor_data.is_null() || state.is_null() {
        return C_GraspResult {
            micro_slip_detected: false,
            macro_slip_detected: false,
            rotational_slip_detected: false,
            commanded_force: 0.0,
            margin: 0.0,
            estimated_mu: 0.0,
        };
    }
    unsafe {
        crate::domains::dexterous::evaluate_grasp_dynamics(&*sensor_data, &mut *state, dt)
    }
}

#[no_mangle]
pub extern "C" fn ztp_surgical_evaluate_grasp(
    auditor: *const C_SurgicalTissueAuditor,
    dt: f32,
) -> C_SurgicalResult {
    if auditor.is_null() {
        return C_SurgicalResult {
            tissue_overstress_detected: false,
            viscoelastic_rupture_detected: false,
            cable_slip_fault: false,
            clamped_force: 0.0,
        };
    }
    unsafe {
        crate::domains::dexterous::evaluate_surgical_grasp_dynamics(&*auditor, dt)
    }
}

#[no_mangle]
pub extern "C" fn ztp_micro_evaluate_release(
    auditor: *const C_MicroReleaseAuditor,
    dt: f32,
) -> C_MicroResult {
    if auditor.is_null() {
        return C_MicroResult {
            release_stiction_active: false,
            electrostatic_charge_violation: false,
            piezo_shake_trigger: false,
            safe_to_retract: false,
        };
    }
    unsafe {
        crate::domains::dexterous::evaluate_micro_release_dynamics(&*auditor, dt)
    }
}

// Expose Directed Energy Laser Targeting FFI wrappers
pub use crate::domains::directed_energy::C_LaserTargetState;

#[no_mangle]
pub extern "C" fn ztp_directed_energy_step(
    state: *mut C_LaserTargetState,
    y_meas: f64,
    dy_history: *const f64,
    dy_history_len: u32,
    apply_ztp: bool,
    dt: f64,
) -> bool {
    if state.is_null() {
        return false;
    }
    unsafe {
        let history = if dy_history.is_null() || dy_history_len == 0 {
            &[]
        } else {
            std::slice::from_raw_parts(dy_history, dy_history_len as usize)
        };
        crate::domains::directed_energy::step_directed_energy(&mut *state, y_meas, history, apply_ztp, dt)
    }
}
// Expose Drone Flight & VSLAM Coherence FFI wrappers
pub use crate::domains::drone::{C_DroneState, C_DroneResult};

#[no_mangle]
pub extern "C" fn ztp_drone_step(
    state: *mut C_DroneState,
    vslam_vel_x: f64,
    vslam_vel_y: f64,
    vslam_vel_z: f64,
    vslam_vel_prev_x: f64,
    vslam_vel_prev_y: f64,
    vslam_vel_prev_z: f64,
    coherence_threshold: f64,
    dt: f64,
) -> C_DroneResult {
    if state.is_null() {
        return C_DroneResult {
            imu_acceleration: [0.0; 3],
            true_acceleration: [0.0; 3],
            coherence_residual: 0.0,
            coherence_fail: false,
        };
    }
    unsafe {
        let vslam_vel = [vslam_vel_x, vslam_vel_y, vslam_vel_z];
        let vslam_vel_prev = [vslam_vel_prev_x, vslam_vel_prev_y, vslam_vel_prev_z];
        crate::domains::drone::step_drone_dynamics(
            &mut *state,
            vslam_vel,
            vslam_vel_prev,
            dt,
            coherence_threshold,
        )
    }
}

// Expose BlueROV2 Marine FFI wrappers
pub use crate::domains::bluerov::{C_BlueRovState, C_BlueRovResult};

#[no_mangle]
pub extern "C" fn ztp_bluerov_step(
    state: *mut C_BlueRovState,
    nav_vel_x: f64,
    nav_vel_y: f64,
    nav_vel_z: f64,
    coherence_threshold: f64,
    dt: f64,
) -> C_BlueRovResult {
    if state.is_null() {
        return C_BlueRovResult {
            imu_acceleration: [0.0; 3],
            true_acceleration: [0.0; 3],
            coherence_residual: 0.0,
            coherence_fail: false,
        };
    }
    unsafe {
        let nav_vel = [nav_vel_x, nav_vel_y, nav_vel_z];
        crate::domains::bluerov::step_bluerov_dynamics(
            &mut *state,
            nav_vel,
            coherence_threshold,
            dt,
        )
    }
}


// ─── UNIT TESTS ──────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::crypto::{Sha256, hex_encode};

    #[test]
    fn test_sha256_empty() {
        let h = Sha256::new();
        assert_eq!(
            hex_encode(&h.finalize()),
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
        );
    }

    #[test]
    fn test_sha256_abc() {
        let mut h = Sha256::new();
        h.update(b"abc");
        assert_eq!(
            hex_encode(&h.finalize()),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
    }

    #[test]
    fn test_sha256_long() {
        let mut h = Sha256::new();
        h.update(b"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq");
        assert_eq!(
            hex_encode(&h.finalize()),
            "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1"
        );
    }

    #[test]
    fn test_drone_coherence() {
        use super::{C_DroneState, ztp_drone_step};
        let mut state = C_DroneState {
            position: [0.0, 0.0, 5.0],
            velocity: [0.0, 0.0, 0.0],
            pitch_roll_yaw: [0.0, 0.0, 0.0],
            motor_rpm: [0.5, 0.5, 0.5, 0.5],
            wind_velocity: [0.0, 0.0, 0.0],
            mass: 1.5,
            drag_coefficient: 0.15,
            max_thrust: 30.0,
        };

        // Nominal step (no visual drift)
        let result = ztp_drone_step(
            &mut state,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            1.5,
            0.001,
        );

        assert!(result.coherence_residual < 1.0);
        assert!(!result.coherence_fail);

        // Anomalous visual drift (VSLAM reports a sudden 10m/s shift in 1ms)
        let result_fail = ztp_drone_step(
            &mut state,
            10.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            1.5,
            0.001,
        );
        assert!(result_fail.coherence_residual > 100.0);
        assert!(result_fail.coherence_fail);
    }

    #[test]
    fn test_bluerov_coherence() {
        use super::{C_BlueRovState, ztp_bluerov_step};
        let mut state = C_BlueRovState {
            position: [0.0, 0.0, -10.0],
            velocity: [1.5, 0.0, 0.0],
            pitch_roll_yaw: [0.0, 0.0, 0.0],
            thruster_commands: [0.5, 0.5, 0.5, 0.5, 0.0, 0.0],
            current_velocity: [0.0, 0.0, 0.0],
            mass: 11.0,
            volume: 0.011,
            drag_coefficients: [0.07, 0.15, 0.20],
            max_thrust_horizontal: 100.0,
            max_thrust_vertical: 100.0,
            tether_anchor: [0.0, 0.0, 0.0],
            tether_length: 50.0,
            tether_k: 0.0,
        };

        // Nominal step (claimed velocity matches actual velocity)
        let result = ztp_bluerov_step(
            &mut state,
            1.5, 0.0, 0.0,
            0.15,
            0.001,
        );
        assert!(result.coherence_residual < 0.1);
        assert!(!result.coherence_fail);

        // Anomaly step (claimed velocity is 5.0m/s which diverges, causing drag mismatch)
        let result_fail = ztp_bluerov_step(
            &mut state,
            5.0, 0.0, 0.0,
            0.15,
            0.001,
        );
        assert!(result_fail.coherence_residual > 0.5);
        assert!(result_fail.coherence_fail);
    }
}

