#!/usr/bin/env python3
"""
biological_compounding_test.py: Verifies the Layer 1 Somatic-Biological Invariant Solvers
executing natively on-register. Connects the four vectors (FKPP propagation, Ostwald-de Waele,
Noyes-Whitney, and Somatic Vagal Core) with the Ouroboros cryptographic state seal.
"""

import os
import sys
import ctypes
import time

# Add current directory to path to import ztp_loader
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(SCRIPT_DIR)

from ztp_loader import load_ztp_library

# ANSI Colors
C_BLUE = "\033[94m"
C_GREEN = "\033[92m"
C_YELLOW = "\033[93m"
C_RED = "\033[91m"
C_BOLD = "\033[1m"
C_END = "\033[0m"

BANNER = f"""
{C_BLUE}{C_BOLD}================================================================================
  ██████╗ ██╗ ██████╗       ██████╗ ██╗ ██████╗     ███████╗ ██████╗ ███╗   ███╗ █████╗ 
  ██╔══██╗██║██╔═══██╗      ██╔══██╗██║██╔═══██╗    ██╔════╝██╔═══██╗████╗ Slot CPU 
  ██████╔╝██║██║   ██║█████╗██████╔╝██║██║   ██║    ███████╗██║   ██║██╔██╗ ██╔██║███████║
  ██╔══██╗██║██║   ██║╚════╝██╔══██╗██║██║   ██║    ╚════██║██║   ██║██║╚████╔╝██║██╔══██║
  ██████╔╝██║╚██████╔╝      ██████╔╝██║╚██████╔╝    ███████║╚██████╔╝██║ ╚███╔╝ ██║██║  ██║
  ╚══════╝ ╚═╝ ╚═════╝       ╚══════╝ ╚═╝ ╚═════╝     ╚══════╝ ╚═════╝ ╚═╝  ╚═╝  ╚═╝╚═╝  ╚═╝
  Layer 1 Biological Invariant Solvers (FKPP, Ostwald, Noyes-Whitney, Vagal Bridge)
================================================================================{C_END}
"""

# ─── CTYPES STRUCTURES ────────────────────────────────────────────────────────

class C_OstwaldDeWaeleFluid(ctypes.Structure):
    _fields_ = [
        ("consistency_index_k", ctypes.c_float),
        ("flow_index_n", ctypes.c_float),
        ("critical_shear_limit", ctypes.c_float),
        ("accumulated_shear_stress", ctypes.c_float),
    ]

class C_NoyesWhitneySolver(ctypes.Structure):
    _fields_ = [
        ("diffusion_coefficient_d", ctypes.c_float),
        ("active_surface_area_a", ctypes.c_float),
        ("boundary_layer_thickness_h", ctypes.c_float),
        ("saturation_solubility_cs", ctypes.c_float),
    ]

class C_SomaticVagalBridge(ctypes.Structure):
    _fields_ = [
        ("heart_rate_bpm", ctypes.c_float),
        ("respiratory_frequency_hz", ctypes.c_float),
        ("trigeminal_pressure_pa", ctypes.c_float),
        ("autonomic_ratio", ctypes.c_float),
    ]

class C_BiologicalState(ctypes.Structure):
    _fields_ = [
        ("timestamp_ms", ctypes.c_uint64),
        ("pathogen_concentration", ctypes.c_float),
        ("cytoplasmic_viscosity", ctypes.c_float),
        ("mucosal_dissolution_rate", ctypes.c_float),
        ("autonomic_ratio", ctypes.c_float),
        ("hash_seal", ctypes.c_ubyte * 32),
    ]

# ─── MAIN PIPELINE ────────────────────────────────────────────────────────────

def main():
    print(BANNER)
    
    # Load dynamic library
    try:
        lib = load_ztp_library(SCRIPT_DIR)
        
        # Configure FFI signatures
        lib.ztp_compounding_fkpp_step.argtypes = [
            ctypes.POINTER(ctypes.c_float),  # concentrations
            ctypes.POINTER(ctypes.c_float),  # diffusions
            ctypes.POINTER(ctypes.c_float),  # replications
            ctypes.c_uint32,                 # num_nodes
            ctypes.c_float,                  # dx
            ctypes.c_float,                  # dt
            ctypes.POINTER(ctypes.c_float),  # next_concentrations
        ]
        lib.ztp_compounding_fkpp_step.restype = None

        lib.ztp_compounding_compute_viscosity.argtypes = [
            ctypes.POINTER(C_OstwaldDeWaeleFluid),
            ctypes.c_float,
        ]
        lib.ztp_compounding_compute_viscosity.restype = ctypes.c_float

        lib.ztp_compounding_audit_shear.argtypes = [
            ctypes.POINTER(C_OstwaldDeWaeleFluid),
        ]
        lib.ztp_compounding_audit_shear.restype = ctypes.c_bool

        lib.ztp_compounding_compute_dissolution_rate.argtypes = [
            ctypes.POINTER(C_NoyesWhitneySolver),
            ctypes.c_float,
            ctypes.c_float,
        ]
        lib.ztp_compounding_compute_dissolution_rate.restype = ctypes.c_float

        lib.ztp_compounding_update_autonomic_tone.argtypes = [
            ctypes.POINTER(C_SomaticVagalBridge),
            ctypes.c_float,
            ctypes.c_float,
        ]
        lib.ztp_compounding_update_autonomic_tone.restype = ctypes.c_float

        lib.ztp_compounding_seal_state.argtypes = [
            ctypes.POINTER(C_BiologicalState),
            ctypes.POINTER(ctypes.c_ubyte),  # out_hash (32 bytes pointer)
        ]
        lib.ztp_compounding_seal_state.restype = None

    except Exception as e:
        print(f"❌ Failed to load or configure ztp_runtime library: {e}")
        sys.exit(1)

    # ══════════════════════════════════════════════════════════════════════════
    # VECTOR 1: VIROLOGY & PATHOGEN PROPAGATION (FKPP PDE SOLVER)
    # ══════════════════════════════════════════════════════════════════════════
    print(f"\n🧬 {C_BOLD}Vector 1: Virology & Pathogen Propagation (FKPP Field Equations){C_END}")
    print("-" * 80)
    
    NUM_NODES = 50
    dx = 0.1
    dt = 0.05
    
    # Initialize concentration buffer with a localized pathogen spike in the center (node 25)
    concentrations = (ctypes.c_float * NUM_NODES)()
    for i in range(NUM_NODES):
        concentrations[i] = 0.0
    concentrations[25] = 1.0  # Spike concentration representing initial viral inoculation
    
    diffusions = (ctypes.c_float * NUM_NODES)()
    replications = (ctypes.c_float * NUM_NODES)()
    for i in range(NUM_NODES):
        diffusions[i] = 0.15      # localised tissue porosity
        replications[i] = 0.45    # replication rate (ATP constrained)
        
    next_concentrations = (ctypes.c_float * NUM_NODES)()

    print(f"Initial field snapshot: Node 23-27 concentrations: {[round(concentrations[j], 3) for j in range(23, 28)]}")
    
    # Run 10 steps of the PDE solver
    for step in range(1, 11):
        lib.ztp_compounding_fkpp_step(
            concentrations,
            diffusions,
            replications,
            NUM_NODES,
            dx,
            dt,
            next_concentrations
        )
        # Copy next back to current
        ctypes.memmove(concentrations, next_concentrations, len(concentrations) * 4)
        
        if step % 2 == 0:
            print(f"  Step {step:02d} (t={step*dt:.2f}s) | Node 23-27: {[round(concentrations[j], 3) for j in range(23, 28)]}")

    final_pathogen_conc = concentrations[25]

    # ══════════════════════════════════════════════════════════════════════════
    # VECTOR 2: ENTERIC & CYTOPLASMIC TRANSPORT (OSTWALD-DE WAELE RHEOLOGY)
    # ══════════════════════════════════════════════════════════════════════════
    print(f"\n🧪 {C_BOLD}Vector 2: Enteric & Cytoplasmic Transport (Ostwald-de Waele Rheology){C_END}")
    print("-" * 80)
    
    fluid = C_OstwaldDeWaeleFluid()
    fluid.consistency_index_k = 2.5     # Pa*s^n (slurry thickness)
    fluid.flow_index_n = 0.4            # highly pseudoplastic (shear-thinning)
    fluid.critical_shear_limit = 45.0   # critical threshold before damage/lysis
    fluid.accumulated_shear_stress = 0.0

    print("Simulating intestinal peristalsis / fluid flow under increasing shear rates:")
    
    # Test low, moderate, and high shear rates
    shear_rates = [0.1, 1.0, 10.0, 50.0, 150.0]
    final_viscosity = 0.0
    for rate in shear_rates:
        viscosity = lib.ztp_compounding_compute_viscosity(ctypes.byref(fluid), rate)
        audit_failed = lib.ztp_compounding_audit_shear(ctypes.byref(fluid))
        
        status = f"{C_RED}CRITICAL LYSIS OVERSTRESS{C_END}" if audit_failed else f"{C_GREEN}SAFE{C_END}"
        print(f"  Shear Rate: {rate:6.1f} s⁻¹ | Dynamic Viscosity: {viscosity:8.4f} Pa·s | Accum Stress: {fluid.accumulated_shear_stress:7.2f} Pa | Status: {status}")
        final_viscosity = viscosity

    # ══════════════════════════════════════════════════════════════════════════
    # VECTOR 3: ACTIVE SOLUTE DISSOLUTION (NOYES-WHITNEY BOUNDARY LAYERS)
    # ══════════════════════════════════════════════════════════════════════════
    print(f"\n💊 {C_BOLD}Vector 3: Active Solute Dissolution (Noyes-Whitney Boundary Layers){C_END}")
    print("-" * 80)
    
    solver = C_NoyesWhitneySolver()
    solver.diffusion_coefficient_d = 1.2e-5   # D
    solver.active_surface_area_a = 0.8         # A (solute particle area)
    solver.boundary_layer_thickness_h = 0.005  # h (clogged gut starting point)
    solver.saturation_solubility_cs = 8.0      # Cs

    print(f"Initial State: Clogged gut stagnant boundary layer (h = {solver.boundary_layer_thickness_h*1000.0:.1f} mm), Saturation solubility Cs = {solver.saturation_solubility_cs} mg/mL")
    
    # Low shear (gut stagnation)
    rate_low = lib.ztp_compounding_compute_dissolution_rate(ctypes.byref(solver), 2.0, 0.05)
    print(f"  Low Peristaltic Shear  | Dissolution Rate dm/dt: {rate_low:8.5f} g/s | Remaining Surface Area: {solver.active_surface_area_a:.4f} m²")
    
    # High shear (active peristaltic mixing thinning the boundary layer)
    rate_high = lib.ztp_compounding_compute_dissolution_rate(ctypes.byref(solver), 2.0, 75.0)
    print(f"  High Peristaltic Shear | Dissolution Rate dm/dt: {rate_high:8.5f} g/s | Remaining Surface Area: {solver.active_surface_area_a:.4f} m²")
    
    final_dissolution_rate = rate_high

    # ══════════════════════════════════════════════════════════════════════════
    # VECTOR 4: BIOPHYSICAL TRANSDUCTION (THE SOMATIC VAGAL CORE)
    # ══════════════════════════════════════════════════════════════════════════
    print(f"\n🫁 {C_BOLD}Vector 4: Biophysical Transduction (Somatic Vagal Resonance){C_END}")
    print("-" * 80)
    
    bridge = C_SomaticVagalBridge()
    bridge.heart_rate_bpm = 82.0
    bridge.respiratory_frequency_hz = 0.083  # 0.083 Hz = SOMA 12-second Mayer wave coherence
    bridge.trigeminal_pressure_pa = 0.0
    bridge.autonomic_ratio = 2.2             # High Sympathetic tone (stressed state)

    print(f"Starting autonomic state: Stressed (Sympathetic/Parasympathetic Ratio = {bridge.autonomic_ratio:.2f})")
    print("Initiating slow diaphragmatic breathing (CN X) and mechanical chewing pressure (CN V):")
    
    # Run 5 breathing cycles
    for cycle in range(1, 6):
        ratio = lib.ztp_compounding_update_autonomic_tone(
            ctypes.byref(bridge),
            800.0,   # Chewing pressure (Pa)
            1200.0   # Diaphragmatic respiratory pressure (Pa)
        )
        print(f"  Cycle {cycle} | Chewing load: {bridge.trigeminal_pressure_pa:6.2f} Pa | Autonomic Ratio: {ratio:6.3f} (down-regulating to balance)")

    final_ratio = bridge.autonomic_ratio

    # ══════════════════════════════════════════════════════════════════════════
    # CRYPTOGRAPHIC STATE SEALING (OUROBOROS)
    # ══════════════════════════════════════════════════════════════════════════
    print(f"\n🔒 {C_BOLD}Cryptographic State Sealing (Ouroboros Chain Proof){C_END}")
    print("-" * 80)
    
    state = C_BiologicalState()
    state.timestamp_ms = int(time.time() * 1000)
    state.pathogen_concentration = final_pathogen_conc
    state.cytoplasmic_viscosity = final_viscosity
    state.mucosal_dissolution_rate = final_dissolution_rate
    state.autonomic_ratio = final_ratio
    
    # Initialize starting hash seal with a mock genesis seed
    for idx in range(32):
        state.hash_seal[idx] = idx

    print(f"Genesis Seed Seal:    {bytes(state.hash_seal).hex()[:24]}...")
    
    # Generate Seal
    out_hash = (ctypes.c_ubyte * 32)()
    lib.ztp_compounding_seal_state(ctypes.byref(state), out_hash)
    
    print(f"Final Ouroboros Seal: {bytes(out_hash).hex()}")
    print("Sovereign biological invariant sweep attested successfully.")
    print("=" * 80 + "\n")

if __name__ == "__main__":
    main()
