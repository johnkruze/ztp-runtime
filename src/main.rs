use std::time::Instant;
use ztp_runtime::domains::terran::{SoilProfile, SoilType, RobotContact, Locomotion};
use ztp_runtime::domains::orbital::{SatelliteState, OrbitalPhysics};
use ztp_runtime::domains::atheric::AthericSystem;
use ztp_runtime::rng::Rng;

fn main() {
    println!("========================================================");
    println!("   ZERO-TRUST PHYSICS RUNTIME (ztp-runtime) BENCHMARK   ");
    println!("========================================================");
    println!("Operational Posture: Bare-Metal / Zero-Dependency");
    println!();

    // ─── 1. TERRAN BENCHMARK ──────────────────────────────────────────
    println!("Running Terran Soil-Dynamics Solver...");
    let terran_iters = 1_000_000;
    let mut soil = SoilProfile {
        soil_type: SoilType::Loam,
        moisture: 0.2,
        glomalin_mg_g: 0.5,
        compaction: 0.1,
        depth_layers: 20,
    };
    let robot = RobotContact {
        mass_kg: 2000.0,
        footprint_m2: 0.25,
        locomotion: Locomotion::Wheeled,
    };

    let start = Instant::now();
    for _ in 0..terran_iters {
        let _res = soil.evaluate_contact(&robot);
        ztp_runtime::domains::terran::moisture_step(&mut soil, 0.01, 0.1);
    }
    let terran_elapsed = start.elapsed();
    let terran_steps_per_sec = terran_iters as f64 / terran_elapsed.as_secs_f64();
    let terran_target = 10750.0;
    let terran_ok = terran_steps_per_sec >= terran_target;

    println!(
        "  -> Iterations: {}\n  -> Elapsed: {:?}\n  -> Throughput: {:.2} steps/s (Target: {:.2} steps/s)\n  -> Status: {}",
        terran_iters,
        terran_elapsed,
        terran_steps_per_sec,
        terran_target,
        if terran_ok { "PASS [OPTIMAL]" } else { "FAIL [SUBOPTIMAL]" }
    );
    println!();

    // ─── 2. ORBITAL BENCHMARK ─────────────────────────────────────────
    println!("Running Orbital 20D Attitude & harmonics Tracker...");
    let orbital_iters = 100_000;
    let mut state = SatelliteState {
        position: [6878.137, 0.0, 0.0],
        velocity: [0.0, 7.612, 0.0],
        quaternion_attitude: [1.0, 0.0, 0.0, 0.0],
        angular_velocity: [0.01, -0.02, 0.005],
        inertia_tensor: [
            [20.0, 0.0, 0.0],
            [0.0, 20.0, 0.0],
            [0.0, 0.0, 30.0],
        ],
    };
    let physics = OrbitalPhysics::default();

    let start = Instant::now();
    for _ in 0..orbital_iters {
        physics.step_6dof(&mut state, 0.1);
        OrbitalPhysics::step_attitude(&mut state, &[0.01, 0.0, -0.01], 0.1);
    }
    let orbital_elapsed = start.elapsed();
    let orbital_steps_per_sec = orbital_iters as f64 / orbital_elapsed.as_secs_f64();
    let orbital_target = 353.0;
    let orbital_ok = orbital_steps_per_sec >= orbital_target;

    println!(
        "  -> Iterations: {}\n  -> Elapsed: {:?}\n  -> Throughput: {:.2} steps/s (Target: {:.2} steps/s)\n  -> Status: {}",
        orbital_iters,
        orbital_elapsed,
        orbital_steps_per_sec,
        orbital_target,
        if orbital_ok { "PASS [OPTIMAL]" } else { "FAIL [SUBOPTIMAL]" }
    );
    println!();

    // ─── 3. ATHERIC BENCHMARK ─────────────────────────────────────────
    println!("Running Atheric RF Pathloss & Coherence Monitor...");
    let atheric_iters = 1_000_000;
    let mut system = AthericSystem::new(8, 10.0, -120.0, 15.0);
    system.hop_seed = [
        0x5a, 0xb8, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc,
        0xde, 0xf0, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc,
        0xde, 0xf0, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc,
        0xde, 0xf0, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc,
    ];
    let mut rng = Rng::new(1337);

    let start = Instant::now();
    for _ in 0..atheric_iters {
        let _res = system.transmit_packet(3.0);
        system.apply_fading(0.05, &mut rng);
    }
    let atheric_elapsed = start.elapsed();
    let atheric_steps_per_sec = atheric_iters as f64 / atheric_elapsed.as_secs_f64();
    let atheric_target = 15830.0;
    let atheric_ok = atheric_steps_per_sec >= atheric_target;

    println!(
        "  -> Iterations: {}\n  -> Elapsed: {:?}\n  -> Throughput: {:.2} steps/s (Target: {:.2} steps/s)\n  -> Status: {}",
        atheric_iters,
        atheric_elapsed,
        atheric_steps_per_sec,
        atheric_target,
        if atheric_ok { "PASS [OPTIMAL]" } else { "FAIL [SUBOPTIMAL]" }
    );
    println!();

    println!("========================================================");
    if terran_ok && orbital_ok && atheric_ok {
        println!("   ALL SYSTEM BENCHMARKS PASSED. SOVEREIGN REALITY CHOSEN.   ");
    } else {
        println!("   BENCHMARK COMPROMISED. OPTIMIZE COMPILATION.   ");
    }
    println!("========================================================");
}
