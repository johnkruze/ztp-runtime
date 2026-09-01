# ztp-runtime — the Reflex

**On-device loop for a VLA / GNC planner.** Zero crate dependencies. Pure Rust stdlib. Compiles to a C library you load next to the motor. Hz is the body’s: grasp is 1 kHz, plasma is 20 Hz, confine is 1 µs.

Your policy thinks at 5–50 Hz. Grasp slip happens in ~2 ms. This crate is the thing that catches it — and the other bodies at their own dt.

```
VLA / GNC  (5–50 Hz)     ztp-runtime  (body Hz)     actuator
      intent      →      friction / stop / force     →   metal
                         projection on-die
```

Sibling: [genesis-core](https://github.com/johnkruze/genesis-core) is the **Forge** (sealed failure-boundary Monte Carlo). This crate is the thin **Reflex** only. Do not merge them.

[zerotrustphysics.com](https://zerotrustphysics.com) · commercial: [Reflex Runtime eval](https://zerotrustphysics.com/offerings#reflex)

---

## Build

```bash
cargo build --release
# macOS  → target/release/libztp_runtime.dylib
# Linux  → target/release/libztp_runtime.so
# Windows → target/release/ztp_runtime.dll

cargo run --release    # microbench
```

Release profile: `opt-level=3`, LTO, `codegen-units=1`, `panic=abort`, strip.

Demo the VLA gap (Python ctypes, this crate):

```bash
cargo build --release
python3 examples/python/vla_somatic_bridge.py
```

C eval box is `grokd/public/ztp-runtime-eval/` — `cd examples && make help && make check`. One dylib, many clocks (grasp 1 kHz · orbit 100 Hz · plasma 20 Hz · hypha 10 Hz · confine 1 µs).

---

## C FFI

All entry points are `#[no_mangle] extern "C"` in `src/lib.rs`. That source is the ABI.

| Export | Domain |
|--------|--------|
| `ztp_dexterous_evaluate_grasp` | Tactile grasp / slip |
| `ztp_dexterous_evaluate_hand` | Serial finger + tendon / pad cone |
| `ztp_surgical_evaluate_grasp` | Surgical force ceiling |
| `ztp_micro_evaluate_release` | Micro-assembly release |
| `ztp_drone_step` | Multirotor step |
| `ztp_bluerov_step` | UUV / ROV step |
| `ztp_marine_evaluate_state` | Mackenzie / hydrostatic / Snell (64 B live orb) |
| `ztp_mycelial_evaluate_state` | Kirchhoff hyphae (64 B live orb, SPECTRA MycelialState) |
| `ztp_last_state_*` | `.soma.bin` header + frame peek / pack (64 B file pinout) |
| `ztp_plasma_fp_vs_l1` | Sheath vs GPS L1, 20 Hz |
| `ztp_tokamak_step` | Bottle, 1 µs |
| `ztp_swing_step` | Grid rotor, 1 ms |
| `ztp_directed_energy_step` | Gimbal / jitter |
| `ztp_terran_evaluate_contact` | Soil contact |
| `ztp_vehicle_hydroplane_step` | Pacejka chassis / hydroplane (1 kHz; not soil) |
| `ztp_tesseract_step` | Duffing IMU firewall (host 1 kHz; ω_n 100 Hz; not machine.c) |
| `ztp_orbital_step_6dof` | Orbital translation |
| `ztp_orbital_step_attitude` | Orbital attitude |
| `ztp_atheric_handshake` | RF coherence |
| `ztp_mars_step` | Mars EDL step |
| `ztp_compounding_fkpp_step` | FKPP |
| `ztp_compounding_compute_viscosity` | Ostwald–de Waele |
| `ztp_compounding_audit_shear` | Shear audit |
| `ztp_compounding_compute_dissolution_rate` | Noyes–Whitney |
| `ztp_compounding_update_autonomic_tone` | Autonomic tone |
| `ztp_compounding_seal_state` | State seal |

`#[repr(C)]` structs sit next to each export. Python `ctypes` layouts: `examples/python/`.

---

## Domains

| Module | Physics |
|--------|---------|
| `dexterous` | Tactile grasp, surgical auditor, micro-release, hand tendon |
| `drone` | Multirotor dynamics step |
| `bluerov` | Underwater ROV step |
| `marine` | Mackenzie 1981, ρgz, thermocline Snell |
| `atheric` | Friis / Shannon / hop seed |
| `terran` | Boussinesq soil contact |
| `vehicle` | Pacejka hydroplane (1 kHz) |
| `orbital` | 6DOF + quaternion attitude |
| `mars` | CO₂ EDL step |
| `directed_energy` | Gimbal / jitter |
| `compounding` | FKPP, Ostwald–de Waele, Noyes–Whitney |
| `plasma` | Sheath vs GPS L1 (20 Hz) |
| `tokamak` | MHD bottle (1 µs) |
| `swing` | Grid rotor (1 ms) |
| `mycelial` | Kirchhoff hyphae (10 Hz) |

`lib.rs` also carries a deterministic LCG and a hand-rolled SHA-256 ProofChain (no `sha2` crate).

---

## Design

- CPU sequential integrators on purpose — single-body edge loops.
- **No Parquet, no corpus, no Monte Carlo.** That is genesis-core (the Forge).
- Not a cloud API. You load a dylib.

---

## License

Dual [MIT](LICENSE-MIT) / [Apache 2.0](LICENSE-APACHE).

[genesis-core](https://github.com/johnkruze/genesis-core) · [ZeroTrustPhysics.com](https://zerotrustphysics.com)
