# ztp-runtime

**Zero-dependency bare-metal physics kernel. C-compatible FFI. Pure Rust standard library.**

`ztp-runtime` is an embeddable physics runtime with no external crate dependencies. It compiles against the Rust standard library only, with aggressive bare-metal optimization profiles. Designed for direct integration into GNC frameworks and real-time control loops where corpus infrastructure overhead is irrelevant — only the physics matters.

This is the kernel layer of the [G^G physics engine](https://github.com/johnkruze/genesis-core). The full corpus pipeline (SHA-256 proof chains, Parquet export, trajectory management) lives in genesis-core. This repo exposes the raw solvers as a C-compatible FFI library.

---

## Repository Structure

```
ztp-runtime/
├── Cargo.toml            # Zero-dependency release profiles
├── README.md
├── LICENSE-APACHE
├── LICENSE-MIT
└── src/
    ├── main.rs           # Benchmark runner
    ├── lib.rs            # C-compatible API declarations
    └── domains/
        ├── terran.rs     # Soil mechanics & robot contact
        ├── orbital.rs    # 20D relativistic attitude tracker
        └── atheric.rs    # RF coherence & channel hopping
```

---

## Physics Domains

### Terran — Soil Mechanics & Robot Contact

Models the physical boundary conditions of soil compaction and seed germination under robot/vehicle contact. Stress distribution governed by **Boussinesq's half-space equation (1885)**.

For a point load $P$ at the surface, vertical stress $\sigma_z$ at depth $z$ and radial distance $r$:

$$\sigma_z(r, z) = \frac{3P}{2\pi} \frac{z^3}{(r^2 + z^2)^{5/2}}$$

For distributed contact footprints, integrated over circular equivalent area $R = \sqrt{A/\pi}$ via Newmark's formula:

$$\sigma_z(0, z) = P_0 \left[ 1 - \frac{z^3}{(R^2 + z^2)^{3/2}} \right]$$

Effective yield stress couples moisture $\theta$ and glomalin $G$ (mycorrhizal glycoprotein):

$$\sigma_{\text{yield}} = \left( \sigma_{\text{base}} + G \cdot c_{\text{glomalin}} \right) \cdot f(\theta)$$

$$\Delta C = \min\left( 0.1 \cdot \frac{\sigma_z - \sigma_{\text{yield}}}{\sigma_{\text{yield}}},\ 0.5 \right) \quad \text{for } \sigma_z > \sigma_{\text{yield}}$$

---

### Orbital — 20D Relativistic Dynamics

Integrates a 20-dimensional state vector: position, velocity, quaternion attitude, angular velocity, and inertia tensor components.

**Translational dynamics** — Yoshida 4th-order symplectic integration with $J_2$–$J_4$ zonal harmonics and first-order Post-Newtonian relativistic corrections:

$$\mathbf{a} = -\frac{\mu}{r^3}\mathbf{r} + \mathbf{a}_{\text{zonal}} + \mathbf{a}_{\text{1PN}}$$

Gravitational potential with zonal harmonics:

$$V(r, \phi) = \frac{\mu}{r} \left[ 1 - \sum_{n=2}^{4} J_n \left(\frac{R_{\text{eq}}}{r}\right)^n P_n(\sin\phi) \right]$$

First-order Post-Newtonian correction:

$$\mathbf{a}_{\text{1PN}} = \frac{\mu}{c^2 r^3} \left[ \left( \frac{4\mu}{r} - v^2 \right) \mathbf{r} + 4(\mathbf{r} \cdot \mathbf{v})\mathbf{v} \right]$$

**Rotational dynamics** — Euler equations with quaternion kinematics:

$$\mathbf{I}\dot{\boldsymbol{\omega}} = \boldsymbol{\tau}_{\text{ext}} - \boldsymbol{\omega} \times (\mathbf{I}\boldsymbol{\omega})$$

$$\dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega} \end{bmatrix}$$

---

### Atheric — RF Coherence & Cryptographic Channel Hopping

Evaluates electromagnetic path loss under broadband interference, multipath fading, and active jammer profiles.

**Friis transmission equation:**

$$P_r = P_t \cdot \left( \frac{\lambda}{4\pi d} \right)^2 \cdot \gamma_{\text{fading}}$$

**Shannon channel capacity:**

$$C = B \log_2\!\left(1 + \text{SNR}_{\text{linear}}\right)$$

$$\text{SNR}_{\text{linear}} = \frac{P_r \cdot \gamma_{\text{fading}}}{N_0 + I_{\text{jammer}}}$$

**Cryptographic channel hopping** — SHA-256 seeded sequence, decoupled from predictable patterns:

$$k = \text{SHA256}(\text{seed} \parallel \text{index}) \bmod N$$

Clock drift desynchronization ($\Delta t \neq 0$) collapses from $N$-channel capacity to $1/N$ random hit probability.

---

## Benchmark

Bare-metal kernel speed — physics computation only, no SHA-256 corpus sealing, no Parquet I/O, no trajectory management. This is the RT integration rate available to a GNC framework calling into the FFI layer.

```bash
cargo run --release
```

*Apple Silicon M-series, macOS*

| Domain | Physics | Kernel Speed | RT Headroom at 1 kHz |
|--------|---------|:------------:|:--------------------:|
| Terran | Boussinesq soil mechanics | 131,314,021 /s | 131,314× |
| Orbital | 20D relativistic 6DOF + attitude | 8,634,335 /s | 8,634× |
| Atheric | RF Shannon capacity + SHA-256 hopping | 6,102,641 /s | 6,102× |

The full genesis-core corpus pipeline (SHA-256 sealed, Parquet export) runs Terran at ~10,750/s. The difference is the overhead of cryptographic proof chains and columnar serialization — irrelevant inside a real-time loop.

---

## C-Compatible FFI

Direct C-linkage integration into legacy GNC stacks (C++, Python ctypes, Ada).

```c
typedef struct {
    double max_compaction;
    double compaction_depth_m;
} C_SoilResult;

typedef struct {
    double position[3];
    double velocity[3];
    double quaternion_attitude[4];
    double angular_velocity[3];
    double inertia_tensor[9];
} C_SatelliteState;

typedef struct {
    bool    success;
    double  resonance;
    double  avg_snr_db;
} C_HandshakeResult;

// Entry points
C_SoilResult ztp_terran_evaluate_contact(
    int soil_type_code, double moisture, double glomalin_mg_g,
    double compaction, unsigned int depth_layers,
    double mass_kg, double footprint_m2, int locomotion_code
);

void ztp_orbital_step_6dof(C_SatelliteState* state, double dt);

void ztp_orbital_step_attitude(
    C_SatelliteState* state,
    double ext_torque_x, double ext_torque_y, double ext_torque_z,
    double dt
);

C_HandshakeResult ztp_atheric_handshake(
    const unsigned char* seed_bytes,  // 32-byte SHA-256 seed
    double strength, double distance_km
);
```

---

## Cargo.toml Profile

```toml
[profile.release]
opt-level      = 3
lto            = true
codegen-units  = 1
panic          = 'abort'
strip          = true
```

---

## License

Dual-licensed under [MIT](LICENSE-MIT) and [Apache 2.0](LICENSE-APACHE).

---

Part of the [G^G Physics](https://github.com/johnkruze/genesis-core) framework · [genesis-core](https://github.com/johnkruze/genesis-core) · [HuggingFace dataset](https://huggingface.co/datasets/spiderpilot89/gg-physical-ground-truth) · [zerotrustphysics.com](https://zerotrustphysics.com)

*John Kruze · [LinkedIn](https://www.linkedin.com/in/john-kruze-34a6683a5/)*
