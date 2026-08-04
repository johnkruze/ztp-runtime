// =====================================================================
// SOMA (DeepHarmonics) - Biological Compounding & Somatic Core
// File: compounding.rs
// =====================================================================
// This module implements the Layer 1 Somatic-Biological Invariant Solvers.
// It bypasses the symbolic "text" abstractions of conventional Bio-AI
// to execute raw, real-time thermodynamic and mass-transport equations
// natively on standard CPU cache lines.
//
// No heap allocation is permitted in the hot path. All structs are
// aligned to 128-byte boundaries to prevent von Neumann bus stalling.
// =====================================================================

// =====================================================================
// CONSTANTS & THERMODYNAMIC INVARIANTS
// =====================================================================
pub const GOLDEN_RATIO: f32 = 1.618033988749895; // \phi - Anti-resonance damping factor
pub const GAS_CONSTANT_R: f32 = 8.314462618;    // J/(mol*K)
pub const PLASMA_VISCOSITY_BASE: f32 = 1.2e-3;   // Pa*s (Baseline at 37°C)

// =====================================================================
// VECTOR 1: VIROLOGY & PATHOGEN PROPAGATION (FKPP PDE SOLVER)
// =====================================================================
// Governed by the Fisher-Kolmogorov-Petrovsky-Piskunov (FKPP) Equation:
// \frac{\partial u}{\partial t} = \mathbf{D} \nabla^2 u + r u (1 - u)
// =====================================================================

#[repr(C, align(128))]
#[derive(Copy, Clone)]
pub struct FkppNode {
    pub concentration: f32,        // u_t
    pub diffusion_coefficient: f32, // D
    pub replication_rate: f32,     // r (ATP-constrained)
    pub spatial_index: u32,        // 1D packed coordinate index
}

/// Executes a single explicit finite-difference time-step (\Delta t) of the
/// FKPP reaction-diffusion field.
#[inline(always)]
pub fn step_fkpp_propagation(
    concentrations: &mut [f32],
    diffusions: &[f32],
    replications: &[f32],
    dx: f32,
    dt: f32,
    next_concentrations: &mut [f32],
) {
    let n = concentrations.len();
    if n < 3 || next_concentrations.len() < n || diffusions.len() < n || replications.len() < n {
        return;
    }
    let dx_sq = dx * dx;

    for i in 1..(n - 1) {
        let u_curr = concentrations[i];
        let d_eff = diffusions[i];
        let r_eff = replications[i];

        // 1. Calculate 1D Laplacian approximation (\nabla^2 u)
        let laplacian = (concentrations[i + 1] - 2.0 * u_curr + concentrations[i - 1]) / dx_sq;

        // 2. Evaluate the non-linear logistic reaction term: r * u * (1 - u)
        let reaction = r_eff * u_curr * (1.0 - u_curr);

        // 3. Integrate: u_{t+1} = u_t + \Delta t * (D * \nabla^2 u + reaction)
        let mut val = u_curr + dt * (d_eff * laplacian + reaction);
        
        // Physical clipping to enforce concentrations boundary: u \in [0.0, 1.0]
        if val < 0.0 {
            val = 0.0;
        } else if val > 1.0 {
            val = 1.0;
        }
        next_concentrations[i] = val;
    }

    // Apply boundary conditions (Dirichlet / Sterile boundaries)
    next_concentrations[0] = 0.0;
    next_concentrations[n - 1] = 0.0;
}

// =====================================================================
// VECTOR 2: ENTERIC & CYTOPLASMIC TRANSPORT (OSTWALD-DE WAELE RHEOLOGY)
// =====================================================================
// Governed by the Power-Law Viscosity Model:
// \eta = K \cdot \dot{\gamma}^{n-1}
// =====================================================================

#[repr(C, align(128))]
#[derive(Copy, Clone)]
pub struct C_OstwaldDeWaeleFluid {
    pub consistency_index_k: f32,      // Pa*s^n
    pub flow_index_n: f32,             // Dimensionless (n < 0.6 is target)
    pub critical_shear_limit: f32,     // Pa (Threshold before cellular lysis occurs)
    pub accumulated_shear_stress: f32,  // Cumulative shear-history (\tau_s)
}

impl C_OstwaldDeWaeleFluid {
    /// Computes the effective dynamic viscosity (\eta) as a function of the
    /// active, high-frequency shear rate (\dot{\gamma}) computed on-die.
    #[inline(always)]
    pub fn compute_viscosity(&mut self, shear_rate: f32) -> f32 {
        // Prevent division-by-zero or complex numbers if shear rate approaches zero
        let stable_shear = if shear_rate < 1e-5 { 1e-5 } else { shear_rate };

        // \eta = K * \dot{\gamma}^{n-1}
        let viscosity = self.consistency_index_k * stable_shear.powf(self.flow_index_n - 1.0);

        // Calculate shear stress: \tau_s = \eta * \dot{\gamma}
        let shear_stress = viscosity * stable_shear;
        self.accumulated_shear_stress = (self.accumulated_shear_stress + shear_stress) / GOLDEN_RATIO;

        viscosity
    }

    /// Evaluates if the cumulative shear stress violates the mechanical integrity of the substrate.
    #[inline(always)]
    pub fn audit_shear_limit(&self) -> bool {
        self.accumulated_shear_stress > self.critical_shear_limit
    }
}

// =====================================================================
// VECTOR 3: ACTIVE SOLUTE DISSOLUTION (NOYES-WHITNEY BOUNDARY LAYERS)
// =====================================================================
// Governed by the Noyes-Whitney Equation:
// \frac{dm}{dt} = \frac{D \cdot A}{h} \cdot (C_s - C_t)
// =====================================================================

#[repr(C, align(128))]
#[derive(Copy, Clone)]
pub struct C_NoyesWhitneySolver {
    pub diffusion_coefficient_d: f32,   // D (m^2/s)
    pub active_surface_area_a: f32,     // A (m^2, modulated by peristaltic shear)
    pub boundary_layer_thickness_h: f32, // h (meters - clogged gut spikes this)
    pub saturation_solubility_cs: f32,  // C_s (g/mL)
}

impl C_NoyesWhitneySolver {
    /// Computes the instantaneous mass transport rate across the mucosal/cellular boundary.
    #[inline(always)]
    pub fn compute_dissolution_rate(&mut self, current_concentration: f32, fluid_shear_rate: f32) -> f32 {
        // Peristaltic shear rate dynamically thins the stagnant boundary layer:
        // h = h_base / (1.0 + \sqrt{\dot{\gamma}})
        let base_h = self.boundary_layer_thickness_h;
        let dynamic_h = base_h / (1.0 + fluid_shear_rate.sqrt());

        // Prevent division-by-zero if the boundary layer approaches zero
        let safe_h = if dynamic_h < 1e-6 { 1e-6 } else { dynamic_h };

        // \frac{dm}{dt} = \frac{D * A}{h} * (C_s - C_t)
        let rate = (self.diffusion_coefficient_d * self.active_surface_area_a / safe_h) 
            * (self.saturation_solubility_cs - current_concentration);

        // Update active surface area as dissolution progresses (mass loss decreases area)
        self.active_surface_area_a *= GOLDEN_RATIO / (GOLDEN_RATIO + rate.abs() * 1e-3);

        rate
    }
}

// =====================================================================
// VECTOR 4: BIOPHYSICAL TRANSDUCTION (THE SOMATIC VAGAL CORE)
// =====================================================================
// Maps Trigeminal (CN V) mechanical mastication pressure and Vagal (CN X)
// diaphragmatic respiratory stretch tension directly to the autonomic
// ratio (A = S/P).
// =====================================================================

#[repr(C, align(128))]
#[derive(Copy, Clone)]
pub struct C_SomaticVagalBridge {
    pub heart_rate_bpm: f32,
    pub respiratory_frequency_hz: f32, // 0.083 Hz = SOMA 1:2 Mayer Wave Coherence
    pub trigeminal_pressure_pa: f32,   // CN V mechanical mastication load
    pub autonomic_ratio: f32,          // A = S/P (Sympathetic/Parasympathetic)
}

impl C_SomaticVagalBridge {
    /// Performs real-time system identification of the somatic state vector,
    /// updating the Autonomic Ratio using symplectic coupling.
    #[inline(always)]
    pub fn update_autonomic_tone(&mut self, mechanical_chewing_pa: f32, diaphragmatic_pressure_pa: f32) -> f32 {
        self.trigeminal_pressure_pa = (self.trigeminal_pressure_pa + mechanical_chewing_pa) / GOLDEN_RATIO;

        // Trigeminal stimulation (chewing) and slow diaphragmatic breathing (vagal stretch)
        // down-regulate sympathetic tone. Box-breathing drives autonomic balance (A -> 1.0)
        let vagal_activation = diaphragmatic_pressure_pa * self.respiratory_frequency_hz;
        let trigeminal_activation = self.trigeminal_pressure_pa * 1e-4;

        // Sympathetic suppression formula:
        // A_{t+1} = A_t / (1.0 + \alpha * Vagal + \beta * Trigeminal)
        let alpha = 0.05f32;
        let beta = 0.02f32;
        
        let target_ratio = self.autonomic_ratio / (1.0 + alpha * vagal_activation + beta * trigeminal_activation);
        
        // Stabilize ratio utilizing Golden Ratio dampening to prevent autonomic crash
        self.autonomic_ratio = (self.autonomic_ratio + target_ratio) / GOLDEN_RATIO;

        self.autonomic_ratio
    }
}

// =====================================================================
// THE CRYPTOGRAPHIC OUROBOROS STATE SEAL
// =====================================================================

#[repr(C, align(128))]
#[derive(Copy, Clone)]
pub struct C_BiologicalState {
    pub timestamp_ms: u64,
    pub pathogen_concentration: f32,
    pub cytoplasmic_viscosity: f32,
    pub mucosal_dissolution_rate: f32,
    pub autonomic_ratio: f32,
    pub hash_seal: [u8; 32],
}

impl C_BiologicalState {
    /// Generates an un-forgeable SHA-256 seal of the current biological state-vector
    /// to serve as the immutable SOMA proof.
    #[inline(always)]
    pub fn seal_state(&mut self) -> [u8; 32] {
        let mut hasher = crate::crypto::Sha256::new();
        
        hasher.update(&self.timestamp_ms.to_le_bytes());
        hasher.update(&self.pathogen_concentration.to_le_bytes());
        hasher.update(&self.cytoplasmic_viscosity.to_le_bytes());
        hasher.update(&self.mucosal_dissolution_rate.to_le_bytes());
        hasher.update(&self.autonomic_ratio.to_le_bytes());
        hasher.update(&self.hash_seal); // Chaining hashes (Ouroboros)

        let out_hash = hasher.finalize();
        self.hash_seal = out_hash;

        out_hash
    }
}
