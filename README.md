# ztp-runtime — the Reflex

**1000 Hz spinal cord for a VLA / GNC planner.** Zero crate dependencies. Pure Rust stdlib. Compiles to a C library you load next to the motor.

Your policy thinks at 5–50 Hz. Physical failure (micro-slip, RF drop, e-brake) happens in ~2 ms. This crate is the thing that catches it.

```
VLA / GNC  (5–50 Hz)     ztp-runtime  (1000 Hz)     actuator
      intent      →      friction / stop / force     →   metal
                         projection on-die
```

Sibling: [genesis-core](https://github.com/johnkruze/genesis-core) is the **Forge** (sealed failure-boundary Monte Carlo). This crate is the thin **Reflex** only. Do not merge them.

[zerotrustphysics.com](https://zerotrustphysics.com) · commercial: [Reflex Runtime eval](https://zerotrustphysics.com/offerings#kernel)

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

12 N policy drops the part. Same command with the reflex on catches micro-slip at **16 ms** and holds (45 N clamp). Other ctypes examples: `examples/python/README.md`. C eval box (hold / tissue / machine) is the Spectrum sibling `grokd/public/ztp-runtime-eval/`.

---

## C FFI

All entry points are `#[no_mangle] extern "C"` in `src/lib.rs`. That source is the ABI.

| Export | Domain |
|--------|--------|
| `ztp_dexterous_evaluate_grasp` | Tactile grasp / slip |
| `ztp_surgical_evaluate_grasp` | Surgical force ceiling |
| `ztp_micro_evaluate_release` | Micro-assembly release |
| `ztp_drone_step` | Multirotor step |
| `ztp_bluerov_step` | UUV / ROV step |
| `ztp_directed_energy_step` | Gimbal / jitter |
| `ztp_terran_evaluate_contact` | Soil contact |
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
| `dexterous` | Tactile grasp, surgical auditor, micro-release |
| `drone` | Multirotor dynamics step |
| `bluerov` | Underwater ROV step |
| `atheric` | Friis / Shannon / hop seed |
| `terran` | Boussinesq soil contact |
| `orbital` | 6DOF + quaternion attitude |
| `mars` | CO₂ EDL step |
| `directed_energy` | Gimbal / jitter |
| `compounding` | FKPP, Ostwald–de Waele, Noyes–Whitney |

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
