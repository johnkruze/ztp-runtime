//! Last-state file pinout — header + frame, 64 B each.
//! Matches `genesis_core::last_state` and `grokd/public/soma/SOMA.md`.
//! Magic lives in the header only. Marine 8×f64 is a live ocean orb, not this file.

pub const MAGIC: [u8; 4] = *b"SOMA";
pub const SPEC_VERSION: u16 = 1;
pub const BODY_OCEAN: u16 = 6;
pub const BODY_DRONE: u16 = 7;
pub const BODY_VEHICLE: u16 = 9; /* STREAM2 Pacejka chassis / hydroplane */
pub const BODY_MYCELIAL: u16 = 8; /* STREAM1 mycelial Kirchhoff terminal */
pub const BODY_PLASMA: u16 = 10; /* STREAM3 reentry / plasma */
pub const BODY_FUSION: u16 = 11; /* STREAM4 fusion tokamak/pit terminal */
pub const BODY_GRASP: u16 = 28;
pub const BODY_HUMANOID: u16 = 30;
pub const BODY_HAND: u16 = 31;
pub const BODY_COMPOUNDING: u16 = 32; /* STREAM5 compounding mill BROTH001 */
pub const FLAG_FUSION_PROMPT: u64 = 1 << 0;
pub const FLAG_FUSION_SURVIVED: u64 = 1 << 1;
pub const FLAG_DARK_WINDOW: u64 = 1 << 0;
pub const FLAG_HUMANOID_BUCKLE: u64 = 1 << 1;
pub const FLAG_HUMANOID_REFLEX: u64 = 1 << 2;
pub const FLAG_HAND_OVERSTRETCH: u64 = 1 << 0;
pub const FLAG_HAND_PAD_SLIP: u64 = 1 << 1;
pub const FLAG_OCEAN_CRUSHED: u64 = 1 << 0;
pub const FLAG_OCEAN_STARVED: u64 = 1 << 1;
pub const FLAG_DRONE_DARK: u64 = 1 << 0;
pub const FLAG_DRONE_VSLAM_FAIL: u64 = 1 << 1;
pub const FLAG_DRONE_REFLEX: u64 = 1 << 2;
pub const FLAG_VEHICLE_HYDROPLANE: u64 = 1 << 0; /* STREAM2 body 9 */
pub const FLAG_VEHICLE_CORNER_LOST: u64 = 1 << 1;
pub const FLAG_VEHICLE_GRIP: u64 = 1 << 2;
pub const FLAG_MYCELIAL_FRAGMENTED: u64 = 1 << 0; /* STREAM1 body 8 */
pub const FLAG_MYCELIAL_BELOW_PERC: u64 = 1 << 1;
pub const FLAG_PLASMA_BLACKOUT: u64 = 1 << 0;
pub const FLAG_PLASMA_MISS: u64 = 1 << 1;
pub const FLAG_PLASMA_GPS_HELD: u64 = 1 << 2;
pub const FLAG_COMPOUNDING_POTENCY_COLLAPSED: u64 = 1 << 0; /* STREAM5 body 32 */
pub const FLAG_COMPOUNDING_DISSOLUTION_STALLED: u64 = 1 << 1;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct C_LastStateHeader {
    pub magic: [u8; 4],
    pub spec_version: u16,
    pub body_id: u16,
    pub traj_count: u64,
    pub frame_count: u64,
    pub digest: [u8; 32],
    pub reserved: [u8; 8],
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct C_LastStateFrame {
    pub t: f64,
    pub pos: [f32; 3],
    pub vel: [f32; 3],
    pub force_torque: f32,
    pub residual: f32,
    pub flags: u64,
    pub proof: [u8; 16],
}

const _: () = {
    assert!(core::mem::size_of::<C_LastStateHeader>() == 64);
    assert!(core::mem::size_of::<C_LastStateFrame>() == 64);
    assert!(core::mem::offset_of!(C_LastStateFrame, flags) == 40);
};

pub fn header_ok(bytes: &[u8]) -> bool {
    if bytes.len() < 64 {
        return false;
    }
    if &bytes[0..4] != MAGIC {
        return false;
    }
    let frames = u64::from_le_bytes(bytes[16..24].try_into().unwrap());
    frames > 0 && bytes.len() as u64 >= 64 + frames * 64
}

pub fn peek_last(bytes: &[u8]) -> Option<C_LastStateFrame> {
    if !header_ok(bytes) {
        return None;
    }
    let frames = u64::from_le_bytes(bytes[16..24].try_into().unwrap()) as usize;
    let off = 64 + (frames - 1) * 64;
    let slice = bytes.get(off..off + 64)?;
    let mut raw = [0u8; 64];
    raw.copy_from_slice(slice);
    Some(unsafe { core::mem::transmute(raw) })
}

pub fn pack_humanoid(
    timestamp_ms: u32,
    com_xyz: [f32; 3],
    velocity_xyz: [f32; 3],
    pitch_rad: f32,
    zmp_margin_m: f32,
    is_dark_window: bool,
    is_buckle: bool,
    is_reflex_grasp: bool,
) -> C_LastStateFrame {
    let mut flags = 0u64;
    if is_dark_window {
        flags |= FLAG_DARK_WINDOW;
    }
    if is_buckle {
        flags |= FLAG_HUMANOID_BUCKLE;
    }
    if is_reflex_grasp {
        flags |= FLAG_HUMANOID_REFLEX;
    }
    let t = timestamp_ms as f64 / 1000.0;
    let mut h = crate::crypto::Sha256::new();
    h.update(&t.to_le_bytes());
    for f in &com_xyz {
        h.update(&f.to_le_bytes());
    }
    for f in &velocity_xyz {
        h.update(&f.to_le_bytes());
    }
    h.update(&pitch_rad.to_le_bytes());
    h.update(&zmp_margin_m.to_le_bytes());
    h.update(&flags.to_le_bytes());
    let digest = h.finalize();
    let mut proof = [0u8; 16];
    proof.copy_from_slice(&digest[..16]);
    C_LastStateFrame {
        t,
        pos: com_xyz,
        vel: velocity_xyz,
        force_torque: pitch_rad,
        residual: zmp_margin_m,
        flags,
        proof,
    }
}

pub fn pack_hand(
    timestamp_ms: u32,
    tension_n: f32,
    pad_normal_n: f32,
    stretch_m: f32,
    opposition_rad: f32,
    q_mcp: f32,
    slip_m_s: f32,
    margin: f32,
    object_span_m: f32,
    tendon_overstretch: bool,
    pad_slip: bool,
) -> C_LastStateFrame {
    let mut flags = 0u64;
    if tendon_overstretch {
        flags |= FLAG_HAND_OVERSTRETCH;
    }
    if pad_slip {
        flags |= FLAG_HAND_PAD_SLIP;
    }
    let t = timestamp_ms as f64 / 1000.0;
    let pos = [tension_n, pad_normal_n, stretch_m];
    let vel = [opposition_rad, q_mcp, slip_m_s];
    let mut h = crate::crypto::Sha256::new();
    h.update(&t.to_le_bytes());
    for f in &pos {
        h.update(&f.to_le_bytes());
    }
    for f in &vel {
        h.update(&f.to_le_bytes());
    }
    h.update(&margin.to_le_bytes());
    h.update(&object_span_m.to_le_bytes());
    h.update(&flags.to_le_bytes());
    let digest = h.finalize();
    let mut proof = [0u8; 16];
    proof.copy_from_slice(&digest[..16]);
    C_LastStateFrame {
        t,
        pos,
        vel,
        force_torque: margin,
        residual: object_span_m,
        flags,
        proof,
    }
}

/// Ocean slots (body 6): pos=depth/pressure/battery_wh, vel=true_crush/believed/used_pct,
/// force_torque=mass_kg, residual=target_depth_m.
pub fn pack_ocean(
    timestamp_ms: u32,
    max_depth_m: f32,
    peak_pressure_mpa: f32,
    battery_wh: f32,
    true_crush_m: f32,
    believed_crush_m: f32,
    battery_used_pct: f32,
    mass_kg: f32,
    target_depth_m: f32,
    is_crushed: bool,
    is_power_starved: bool,
) -> C_LastStateFrame {
    let mut flags = 0u64;
    if is_crushed {
        flags |= FLAG_OCEAN_CRUSHED;
    }
    if is_power_starved {
        flags |= FLAG_OCEAN_STARVED;
    }
    let t = timestamp_ms as f64 / 1000.0;
    let pos = [max_depth_m, peak_pressure_mpa, battery_wh];
    let vel = [true_crush_m, believed_crush_m, battery_used_pct];
    let mut h = crate::crypto::Sha256::new();
    h.update(&t.to_le_bytes());
    for f in &pos {
        h.update(&f.to_le_bytes());
    }
    for f in &vel {
        h.update(&f.to_le_bytes());
    }
    h.update(&mass_kg.to_le_bytes());
    h.update(&target_depth_m.to_le_bytes());
    h.update(&flags.to_le_bytes());
    let digest = h.finalize();
    let mut proof = [0u8; 16];
    proof.copy_from_slice(&digest[..16]);
    C_LastStateFrame {
        t,
        pos,
        vel,
        force_torque: mass_kg,
        residual: target_depth_m,
        flags,
        proof,
    }
}

/// Drone file slots (body 7): pos=xyz, vel=vxyz, force_torque=pitch, residual=coherence.
pub fn pack_drone(
    timestamp_ms: u32,
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    vel_x: f32,
    vel_y: f32,
    vel_z: f32,
    pitch_rad: f32,
    coherence_residual: f32,
    is_dark_window: bool,
    is_vslam_fail: bool,
    is_reflex_active: bool,
) -> C_LastStateFrame {
    let mut flags = 0u64;
    if is_dark_window {
        flags |= FLAG_DRONE_DARK;
    }
    if is_vslam_fail {
        flags |= FLAG_DRONE_VSLAM_FAIL;
    }
    if is_reflex_active {
        flags |= FLAG_DRONE_REFLEX;
    }
    let t = timestamp_ms as f64 / 1000.0;
    let pos = [pos_x, pos_y, pos_z];
    let vel = [vel_x, vel_y, vel_z];
    let mut h = crate::crypto::Sha256::new();
    h.update(&t.to_le_bytes());
    for f in &pos {
        h.update(&f.to_le_bytes());
    }
    for f in &vel {
        h.update(&f.to_le_bytes());
    }
    h.update(&pitch_rad.to_le_bytes());
    h.update(&coherence_residual.to_le_bytes());
    h.update(&flags.to_le_bytes());
    let digest = h.finalize();
    let mut proof = [0u8; 16];
    proof.copy_from_slice(&digest[..16]);
    C_LastStateFrame {
        t,
        pos,
        vel,
        force_torque: pitch_rad,
        residual: coherence_residual,
        flags,
        proof,
    }
}

/// Fusion pit slots (body 11): pos=flux/beta_eff/time_s, vel=xenon_worth/pit_hours/core_age_days,
/// force_torque=delta_rho, residual=base_rho. Flags bit0 prompt-critical · bit1 pit-survived.
/// STREAM4 — reactor hours stay a terminal frame; no hour loop.
pub fn pack_fusion(
    t_s: f64,
    flux: f32,
    beta_eff: f32,
    time_s: f32,
    xenon_worth: f32,
    pit_hours: f32,
    core_age_days: f32,
    delta_rho: f32,
    base_rho: f32,
    is_prompt_critical: bool,
    is_pit_survived: bool,
) -> C_LastStateFrame {
    let mut flags = 0u64;
    if is_prompt_critical {
        flags |= FLAG_FUSION_PROMPT;
    }
    if is_pit_survived {
        flags |= FLAG_FUSION_SURVIVED;
    }
    let pos = [flux, beta_eff, time_s];
    let vel = [xenon_worth, pit_hours, core_age_days];
    let mut h = crate::crypto::Sha256::new();
    h.update(&t_s.to_le_bytes());
    for f in &pos {
        h.update(&f.to_le_bytes());
    }
    for f in &vel {
        h.update(&f.to_le_bytes());
    }
    h.update(&delta_rho.to_le_bytes());
    h.update(&base_rho.to_le_bytes());
    h.update(&flags.to_le_bytes());
    let digest = h.finalize();
    let mut proof = [0u8; 16];
    proof.copy_from_slice(&digest[..16]);
    C_LastStateFrame {
        t: t_s,
        pos,
        vel,
        force_torque: delta_rho,
        residual: base_rho,
        flags,
        proof,
    }
}

/// Plasma file slots (body 10): pos=last_gps_x/alt/tgt, vel=fp_ghz/L1_ghz/miss_m,
/// force_torque=fp/L1, residual=miss_m. Flags bit0 blackout · bit1 miss · bit2 GPS-held.
/// STREAM3 — 20 Hz pinout. Not the 128 B HGV Forge cache.
pub fn pack_plasma(
    timestamp_ms: u32,
    last_gps_x_m: f32,
    altitude_m: f32,
    last_gps_tgt_m: f32,
    fp_ghz: f32,
    l1_ghz: f32,
    miss_m: f32,
    fp_over_l1: f32,
    miss_repeat_m: f32,
    is_blackout: bool,
    is_miss: bool,
    is_gps_held: bool,
) -> C_LastStateFrame {
    let mut flags = 0u64;
    if is_blackout {
        flags |= FLAG_PLASMA_BLACKOUT;
    }
    if is_miss {
        flags |= FLAG_PLASMA_MISS;
    }
    if is_gps_held {
        flags |= FLAG_PLASMA_GPS_HELD;
    }
    let t = timestamp_ms as f64 / 1000.0;
    let pos = [last_gps_x_m, altitude_m, last_gps_tgt_m];
    let vel = [fp_ghz, l1_ghz, miss_m];
    let mut h = crate::crypto::Sha256::new();
    h.update(&t.to_le_bytes());
    for f in &pos {
        h.update(&f.to_le_bytes());
    }
    for f in &vel {
        h.update(&f.to_le_bytes());
    }
    h.update(&fp_over_l1.to_le_bytes());
    h.update(&miss_repeat_m.to_le_bytes());
    h.update(&flags.to_le_bytes());
    let digest = h.finalize();
    let mut proof = [0u8; 16];
    proof.copy_from_slice(&digest[..16]);
    C_LastStateFrame {
        t,
        pos,
        vel,
        force_torque: fp_over_l1,
        residual: miss_repeat_m,
        flags,
        proof,
    }
}

/// STREAM1 mycelial file slots (body 8): pos=health/density/percolation,
/// vel=delivered/conductance/tilling, force_torque=delivered (repeat),
/// residual=percolation (repeat). Flags bit0 fragmented · bit1 below-percolation.
/// Live 8×f64 orb is C_MycelialState / SPECTRA MycelialState — not this envelope.
pub fn pack_mycelial(
    timestamp_ms: u32,
    health_index: f32,
    hyphal_density: f32,
    percolation_index: f32,
    delivered_nutrient: f32,
    conductance_mean: f32,
    tilling_stress: f32,
    is_fragmented: bool,
    is_below_percolation: bool,
) -> C_LastStateFrame {
    let mut flags = 0u64;
    if is_fragmented {
        flags |= FLAG_MYCELIAL_FRAGMENTED;
    }
    if is_below_percolation {
        flags |= FLAG_MYCELIAL_BELOW_PERC;
    }
    let t = timestamp_ms as f64 / 1000.0;
    let pos = [health_index, hyphal_density, percolation_index];
    let vel = [delivered_nutrient, conductance_mean, tilling_stress];
    let mut h = crate::crypto::Sha256::new();
    h.update(&t.to_le_bytes());
    for f in &pos {
        h.update(&f.to_le_bytes());
    }
    for f in &vel {
        h.update(&f.to_le_bytes());
    }
    h.update(&delivered_nutrient.to_le_bytes());
    h.update(&percolation_index.to_le_bytes());
    h.update(&flags.to_le_bytes());
    let digest = h.finalize();
    let mut proof = [0u8; 16];
    proof.copy_from_slice(&digest[..16]);
    C_LastStateFrame {
        t,
        pos,
        vel,
        force_torque: delivered_nutrient,
        residual: percolation_index,
        flags,
        proof,
    }
}

/// STREAM5 compounding mill slots (body 32): pos=acc_shear/potency/dissolution,
/// vel=viscosity/api/shear_rate, force_torque=acc_shear, residual=potency.
/// Flags bit0 potency-collapsed · bit1 dissolution-stalled. Reserved BROTH001.
pub fn pack_compounding(
    timestamp_ms: u32,
    accumulated_shear_stress_pa: f32,
    active_potency_pct: f32,
    dissolution_pct: f32,
    final_viscosity_pas: f32,
    final_api_concentration_kg_m3: f32,
    shear_rate_s1: f32,
    is_potency_collapsed: bool,
    is_dissolution_stalled: bool,
) -> C_LastStateFrame {
    let mut flags = 0u64;
    if is_potency_collapsed {
        flags |= FLAG_COMPOUNDING_POTENCY_COLLAPSED;
    }
    if is_dissolution_stalled {
        flags |= FLAG_COMPOUNDING_DISSOLUTION_STALLED;
    }
    let t = timestamp_ms as f64 / 1000.0;
    let pos = [
        accumulated_shear_stress_pa,
        active_potency_pct,
        dissolution_pct,
    ];
    let vel = [
        final_viscosity_pas,
        final_api_concentration_kg_m3,
        shear_rate_s1,
    ];
    let mut h = crate::crypto::Sha256::new();
    h.update(&t.to_le_bytes());
    for f in &pos {
        h.update(&f.to_le_bytes());
    }
    for f in &vel {
        h.update(&f.to_le_bytes());
    }
    h.update(&accumulated_shear_stress_pa.to_le_bytes());
    h.update(&active_potency_pct.to_le_bytes());
    h.update(&flags.to_le_bytes());
    let digest = h.finalize();
    let mut proof = [0u8; 16];
    proof.copy_from_slice(&digest[..16]);
    C_LastStateFrame {
        t,
        pos,
        vel,
        force_torque: accumulated_shear_stress_pa,
        residual: active_potency_pct,
        flags,
        proof,
    }
}

/// STREAM2 vehicle slots (body 9): pos=μ / |y| / yaw, vel=vx / vy / yaw_rate,
/// force_torque=yaw, residual=|y|. Flags bit0 hydroplane · bit1 corner-lost · bit2 grip.
/// 128 B Forge VehicleDynamicsState is the RAM cache line. This envelope is 64 B.
pub fn pack_vehicle(
    timestamp_ms: u32,
    mu: f32,
    abs_y_m: f32,
    yaw_rad: f32,
    vel_x: f32,
    vel_y: f32,
    yaw_rate: f32,
    is_hydroplane: bool,
    is_corner_lost: bool,
    is_grip: bool,
) -> C_LastStateFrame {
    let mut flags = 0u64;
    if is_hydroplane {
        flags |= FLAG_VEHICLE_HYDROPLANE;
    }
    if is_corner_lost {
        flags |= FLAG_VEHICLE_CORNER_LOST;
    }
    if is_grip {
        flags |= FLAG_VEHICLE_GRIP;
    }
    let t = timestamp_ms as f64 / 1000.0;
    let pos = [mu, abs_y_m, yaw_rad];
    let vel = [vel_x, vel_y, yaw_rate];
    let mut h = crate::crypto::Sha256::new();
    h.update(&t.to_le_bytes());
    for f in &pos {
        h.update(&f.to_le_bytes());
    }
    for f in &vel {
        h.update(&f.to_le_bytes());
    }
    h.update(&yaw_rad.to_le_bytes());
    h.update(&abs_y_m.to_le_bytes());
    h.update(&flags.to_le_bytes());
    let digest = h.finalize();
    let mut proof = [0u8; 16];
    proof.copy_from_slice(&digest[..16]);
    C_LastStateFrame {
        t,
        pos,
        vel,
        force_torque: yaw_rad,
        residual: abs_y_m,
        flags,
        proof,
    }
}

fn frame_to_bytes(frame: C_LastStateFrame) -> [u8; 64] {
    unsafe { core::mem::transmute(frame) }
}

/// Header + frames. Digest is SHA-256 of concatenated frame bytes (SOMA.md).
/// STREAM3 plasma write — same layout as genesis_core::last_state::write_soma_file.
pub fn write_soma_file(body_id: u16, reserved: [u8; 8], frames: &[C_LastStateFrame]) -> Vec<u8> {
    let mut payload = Vec::with_capacity(frames.len() * 64);
    for f in frames {
        payload.extend_from_slice(&frame_to_bytes(*f));
    }
    let mut h = crate::crypto::Sha256::new();
    h.update(&payload);
    let digest = h.finalize();
    let header = C_LastStateHeader {
        magic: MAGIC,
        spec_version: SPEC_VERSION,
        body_id,
        traj_count: 1,
        frame_count: frames.len() as u64,
        digest,
        reserved,
    };
    let hdr: [u8; 64] = unsafe { core::mem::transmute(header) };
    let mut bin = Vec::with_capacity(64 + payload.len());
    bin.extend_from_slice(&hdr);
    bin.extend_from_slice(&payload);
    bin
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sizes() {
        assert_eq!(core::mem::size_of::<C_LastStateHeader>(), 64);
        assert_eq!(core::mem::size_of::<C_LastStateFrame>(), 64);
    }

    #[test]
    fn plasma_file_has_header_body_id_and_frame_digest() {
        let frame = pack_plasma(
            50, 120.0, 0.0, 80.0, 2.1, 1.575, 55.0, 1.33, 55.0, true, true, false,
        );
        assert!((frame.pos[0] - 120.0).abs() < 1e-3);
        assert_eq!(frame.flags & FLAG_PLASMA_BLACKOUT, FLAG_PLASMA_BLACKOUT);
        assert_eq!(frame.flags & FLAG_PLASMA_MISS, FLAG_PLASMA_MISS);
        assert_eq!(frame.flags & FLAG_PLASMA_GPS_HELD, 0);
        let file = write_soma_file(BODY_PLASMA, *b"PLASMA01", &[frame]);
        assert_eq!(&file[0..4], b"SOMA");
        let body = u16::from_le_bytes([file[6], file[7]]);
        assert_eq!(body, BODY_PLASMA);
        assert_ne!(&file[64..68], b"SOMA");
    }

    #[test]
    fn vehicle_file_has_header_body_id_and_frame_digest() {
        /* STREAM2 body 9 — 64 B file, not the 128 B Forge cache line */
        let frame = pack_vehicle(
            4999, 0.148, 3.2, 0.12, 41.0, 0.4, 0.02, true, true, false,
        );
        assert!((frame.pos[0] - 0.148).abs() < 1e-5);
        assert_eq!(frame.flags & FLAG_VEHICLE_HYDROPLANE, FLAG_VEHICLE_HYDROPLANE);
        assert_eq!(frame.flags & FLAG_VEHICLE_CORNER_LOST, FLAG_VEHICLE_CORNER_LOST);
        assert_eq!(frame.flags & FLAG_VEHICLE_GRIP, 0);
        let file = write_soma_file(BODY_VEHICLE, *b"VEHICLE1", &[frame]);
        assert_eq!(&file[0..4], b"SOMA");
        let body = u16::from_le_bytes([file[6], file[7]]);
        assert_eq!(body, BODY_VEHICLE);
        assert_ne!(&file[64..68], b"SOMA");
    }
}
