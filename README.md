# ztp-runtime

**Zero-dependency bare-metal physics kernel · C FFI · Pure Rust stdlib**

Edge Layer 0 for Zero-Trust Physics: project commands onto physical invariants inside real-time loops. No crate dependencies. Compiles to a C-compatible dynamic library for GNC stacks and Python `ctypes`.

Sibling of [genesis-core](https://github.com/johnkruze/genesis-core) (Monte Carlo / sealed trajectory banks). This crate is the thin embeddable reflex kernel only.

[zerotrustphysics.com](https://zerotrustphysics.com) · [spiderpilot89](https://huggingface.co/spiderpilot89)

---

## Layout

```
ztp-runtime/
├── Cargo.toml
├── README.md
├── src/
│   ├── lib.rs          # RNG, zero-dep SHA-256 / ProofChain, C exports
│   ├── main.rs         # Microbench runner
│   └── domains/        # Physics modules
└── target/release/
    └── libztp_runtime.*
```

---

## Domains

| Domain | Module | What it models |
|--------|--------|----------------|
| Terran | `domains/terran.rs` | Boussinesq soil stress, moisture / glomalin, robot contact |
| Orbital | `domains/orbital.rs` | 6DOF translation + quaternion attitude, zonal harmonics |
| Atheric | `domains/atheric.rs` | Friis path loss, Shannon capacity, cryptographic hop seed |
| Mars EDL | `domains/mars.rs` | CO₂ atmosphere, drag, retro-propulsion step |
| Dexterous | `domains/dexterous.rs` | Tactile grasp, surgical tissue auditor, micro-release |
| Directed energy | `domains/directed_energy.rs` | Gimbal / jitter integration step |
| Drone | `domains/drone.rs` | Multirotor dynamics step |
| Subsea ROV | `domains/bluerov.rs` | Underwater ROV dynamics step |
| Compounding | `domains/compounding.rs` | FKPP, Ostwald–de Waele viscosity, Noyes–Whitney dissolution, autonomic tone, state seal |

Internal utilities in `lib.rs`: deterministic LCG PRNG; hand-rolled SHA-256 and proof sealing (no `sha2` crate).

---

## Build

```bash
cargo build --release
cargo run --release          # microbench (terran / orbital / atheric)
```

Artifacts:

| Platform | Library |
|----------|---------|
| macOS | `target/release/libztp_runtime.dylib` |
| Linux | `target/release/libztp_runtime.so` |
| Windows | `target/release/ztp_runtime.dll` |

`Cargo.toml` release profile: `opt-level=3`, LTO, `codegen-units=1`, `panic=abort`, strip.

---

## C FFI index

All entry points are `#[no_mangle] extern "C"` in `src/lib.rs`. Load the dylib and call by name (or bind headers from these signatures).

| Export | Domain |
|--------|--------|
| `ztp_terran_evaluate_contact` | Terran soil contact |
| `ztp_orbital_step_6dof` | Orbital translation |
| `ztp_orbital_step_attitude` | Orbital attitude |
| `ztp_atheric_handshake` | RF coherence handshake |
| `ztp_mars_step` | Mars EDL step |
| `ztp_dexterous_evaluate_grasp` | Tactile grasp |
| `ztp_surgical_evaluate_grasp` | Surgical force ceiling |
| `ztp_micro_evaluate_release` | Micro-assembly release |
| `ztp_directed_energy_step` | Directed-energy gimbal |
| `ztp_drone_step` | Drone step |
| `ztp_bluerov_step` | Subsea ROV step |
| `ztp_compounding_fkpp_step` | Compounding FKPP |
| `ztp_compounding_compute_viscosity` | Ostwald–de Waele |
| `ztp_compounding_audit_shear` | Shear audit |
| `ztp_compounding_compute_dissolution_rate` | Noyes–Whitney |
| `ztp_compounding_update_autonomic_tone` | Autonomic tone |
| `ztp_compounding_seal_state` | State seal |

Structs are `#[repr(C)]` next to each export in `lib.rs` / domain modules. That source is the authoritative ABI.

**Python:** load `libztp_runtime` via `ctypes` with matching `Structure` layouts (see sibling [zero-trust-physics](https://github.com/johnkruze/zero-trust-physics) loaders).

---

## Design notes

- **CPU sequential integrators** on purpose for single-body edge loops.  
- **No corpus / Parquet / trajectory bank** here — that is genesis-core.  
- **Not merged** with genesis_core; keep as a thin embeddable kernel.  
- Microbench in `main.rs` covers three baseline domains; expanding benches is a later task.

---

## License

Dual [MIT](LICENSE-MIT) / [Apache 2.0](LICENSE-APACHE).

---

[genesis-core](https://github.com/johnkruze/genesis-core) · [zero-trust-physics](https://github.com/johnkruze/zero-trust-physics) · [ZeroTrustPhysics.com](https://zerotrustphysics.com)
