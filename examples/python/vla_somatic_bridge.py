#!/usr/bin/env python3
"""
G^G Somatic Physics-Injection: VLA Reflex Grounding Bridge.
Bridges a low-frequency (5Hz) Vision-Language-Action (VLA) policy with the 1000Hz somatic reflex FFI runtime.

This example solves a critical performance bottleneck in modern AI robotics:
Deep neural networks (typical VLA policies) have high inference latency
(typically 100ms - 300ms), making them mathematically incapable of responding to 
high-frequency physical transients (like slip or stiction) in real time.

By placing the Rust-compiled Somatic Grounding Loop on the low-level edge microcontroller (1000Hz),
we implement an "autonomic reflex arc" (a robotic spinal cord) that maintains physical
coherence and stabilizes grasps during the VLA's "cognitive silence" windows.
"""

import os
import sys
import ctypes
import json
import hashlib
import numpy as np

# ANSI Colors
C_BLUE = "\033[94m"
C_GREEN = "\033[92m"
C_YELLOW = "\033[93m"
C_RED = "\033[91m"
C_BOLD = "\033[1m"
C_END = "\033[0m"

BANNER = f"""
{C_BLUE}{C_BOLD}================================================================================
  ██████╗ ██████╗      ██████╗ ██████╗ ██╗██████╗  ██████╗ ███████╗
  ╚════██╗╚════██╗    ██╔════╝ ██╔══██╗██║██╔══██╗██╔════╝ ██╔════╝
   █████╔╝  █████╔╝    ██║  ███╗██████╔╝██║██║  ██║██║  ███╗█████╗  
   ██╔══╝  ██╔═══╝     ██║   ██║██╔══██╗██║██║  ██║██║   ██║██╔══╝  
   ███████╗███████╗    ╚██████╔╝██║  ██║██║██████╔╝╚██████╔╝███████╗
   ╚══════╝╚══════╝     ╚═════╝ ╚═╝  ╚═╝╚═╝╚═════╝  ╚═════╝ ╚══════╝
   Somatic Physics-Injection: VLA (5Hz) to Spinal Reflex (1000Hz) Bridge
================================================================================{C_END}
"""

# ─── CTYPES STRUCTURES (Must match Rust ztp-runtime/src/domains/dexterous.rs) ─

class C_Taxel(ctypes.Structure):
    _fields_ = [
        ("normal", ctypes.c_float),
        ("shear_x", ctypes.c_float),
        ("shear_y", ctypes.c_float),
    ]

class C_TactileArray(ctypes.Structure):
    _fields_ = [
        ("taxels", C_Taxel * 16),
    ]

class C_GraspState(ctypes.Structure):
    _fields_ = [
        ("normal_force", ctypes.c_float),
        ("slip_velocity", ctypes.c_float),
        ("slip_angular_velocity", ctypes.c_float),
        ("object_mass", ctypes.c_float),
        ("static_friction_coeff", ctypes.c_float),
        ("dynamic_friction_coeff", ctypes.c_float),
        ("reflex_active", ctypes.c_bool),
    ]

class C_GraspResult(ctypes.Structure):
    _fields_ = [
        ("micro_slip_detected", ctypes.c_bool),
        ("macro_slip_detected", ctypes.c_bool),
        ("rotational_slip_detected", ctypes.c_bool),
        ("commanded_force", ctypes.c_float),
        ("margin", ctypes.c_float),
        ("estimated_mu", ctypes.c_float),
    ]

# ─── FFI LIBRARY LOADER ───────────────────────────────────────────────────────

try:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Ensure parent/current directories are in path for importing ztp_loader
    sys.path.insert(0, script_dir)
    from ztp_loader import load_ztp_library
    _lib = load_ztp_library(script_dir)
    _lib.ztp_dexterous_evaluate_grasp.argtypes = [
        ctypes.POINTER(C_TactileArray),
        ctypes.POINTER(C_GraspState),
        ctypes.c_float
    ]
    _lib.ztp_dexterous_evaluate_grasp.restype = C_GraspResult
    HAS_ZTP = True
except Exception as e:
    print(f"❌ Failed to load ZTP library: {e}")
    HAS_ZTP = False

# ─── SIMULATION MODEL ─────────────────────────────────────────────────────────

def distribute_forces(normal_force, shear_force, sensor_array):
    """Distributes forces evenly over the 16 taxels of the flat array."""
    for i in range(16):
        sensor_array.taxels[i].normal = normal_force / 16.0
        sensor_array.taxels[i].shear_x = 0.0
        sensor_array.taxels[i].shear_y = (shear_force / 16.0)

def simulate_vla_bridge(apply_ztp):
    """
    Simulates a humanoid hand controlled by a 5Hz VLA policy.
    Total duration: 1.0s (1000 ticks at 1000Hz).
    At t = 0.3s: Payload weight shifts (e.g. robot lifts arm, increasing gravity load to 18N).
    At t = 0.5s: Grasp surface gets slippery (friction drops from 0.45 to 0.22).
    
    Without Somatic Reflex: The hand blindly executes the 5Hz VLA command. Slip occurs, and the payload is dropped.
    With Somatic Reflex: Low-latency reflex corrects normal force in 2ms, preserving the grasp.
    """
    mode_str = "SOMATIC REFLEX: ENABLED" if apply_ztp else "SOMATIC REFLEX: DISABLED"
    print(f"\n{C_BOLD}Attuning VLA Controller Interface ({mode_str}){C_END}")
    print("-" * 90)

    # Initial states
    vla_hz = 5.0
    vla_ticks = int(1000 / vla_hz) # VLA updates every 200ms (200 ticks)
    
    mass = 1.0 # 1.0 kg payload
    g = 9.81
    
    # Tactile and grip state variables
    current_normal_force = 12.0 # N (VLA nominal starting command)
    slip_velocity = 0.0
    y_payload = 0.0 # vertical offset of payload
    v_payload = 0.0
    
    state = C_GraspState()
    state.normal_force = current_normal_force
    state.slip_velocity = 0.0
    state.slip_angular_velocity = 0.0
    state.object_mass = mass
    state.static_friction_coeff = 0.45
    state.dynamic_friction_coeff = 0.35
    state.reflex_active = False
    
    sensor = C_TactileArray()
    
    log = []
    dropped = False
    reflex_triggered = False
    
    vla_reported = False
    
    for step in range(1000):
        t = step * 0.001 # 1ms steps
        
        # 1. Simulate VLA slow inference loop (executes every 200ms)
        is_vla_tick = (step % vla_ticks == 0)
        if is_vla_tick:
            # The VLA wakes up, inspects visual cameras, and outputs target force.
            # In a nominal stack, it commands a constant 12.0N because it doesn't see micro-slip.
            vla_target_force = 12.0
            if not vla_reported and step > 0:
                vla_reported = True
                print(f"🧠 {C_BLUE}[t={t:.2f}s] VLA Inference Cycle: Commands nominal hold force = {vla_target_force} N.{C_END}")
        
        # 2. Dynamic Physical Disturbances
        # Event A: Weight acceleration shift at t = 0.3s (lifting payload)
        if t >= 0.3:
            effective_gravity = g + 8.0 # sudden vertical acceleration
            if t == 0.3:
                print(f"💥 {C_RED}[t={t:.2f}s] ACCELERATION SHIFT! Effective payload load surges to {(mass * effective_gravity):.2f} N.{C_END}")
        else:
            effective_gravity = g
            
        # Event B: Surface slickness transition at t = 0.5s (oily patch)
        if t >= 0.5:
            current_mu = 0.22
            if t == 0.5:
                print(f"💥 {C_RED}[t={t:.2f}s] SURFACE FRICTION DROP! Static friction coefficient collapses to {current_mu:.2f}.{C_END}")
        else:
            current_mu = 0.45
            
        state.static_friction_coeff = current_mu
        state.dynamic_friction_coeff = current_mu * 0.8
        
        # 3. Simulate sensor array physical distribution
        shear_force = mass * effective_gravity
        distribute_forces(current_normal_force, shear_force, sensor)
        
        # 4. Low-Latency Somatic Reflex (Runs at 1000Hz on FFI)
        if apply_ztp:
            # Call Rust native evaluator
            result = _lib.ztp_dexterous_evaluate_grasp(ctypes.byref(sensor), ctypes.byref(state), 0.001)
            
            # The Somatic reflex dynamically calibrates the actuator command
            current_normal_force = result.commanded_force
            
            if result.micro_slip_detected and not reflex_triggered:
                reflex_triggered = True
                print(f"⚡ {C_GREEN}{C_BOLD}[t={t:.3f}s] SOMATIC REFLEX TRIGGERED! Micro-slip detected (friction margin = {result.margin:.2f}).{C_END}")
                print(f"   ├─ Action: Spinal integration overriding low-frequency planning.")
                print(f"   └─ Status: Attuning grip force to stabilize grasp (Force -> {current_normal_force:.2f} N).")
        else:
            # Without ZTP, the controller blindly tracks the slow VLA command
            if is_vla_tick:
                current_normal_force = vla_target_force
                
        # 5. Integrate Physics (Grasp Friction Dynamics)
        friction_limit = 2.0 * current_normal_force * current_mu # 2 contact fingers
        
        # If payload gravity load exceeds maximum friction, macro-slip starts
        if shear_force > friction_limit:
            slip_acc = (shear_force - friction_limit) / mass
            v_payload += slip_acc * 0.001
            y_payload += v_payload * 0.001
            state.slip_velocity = v_payload
        else:
            # Friction holds; decelerate slip velocity (damping)
            v_payload = max(0.0, v_payload - 10.0 * 0.001)
            y_payload = max(0.0, y_payload)
            state.slip_velocity = v_payload
            
        # Check if the payload slipped out of the fingers (>10 cm displacement)
        if y_payload > 0.10 and not dropped:
            dropped = True
            print(f"❌ {C_RED}{C_BOLD}[t={t:.3f}s] PAYLOAD DROPPED! Slip displacement ({y_payload*100:.1f} cm) exceeds grasp limits.{C_END}")
            break
            
        log.append({
            "t": t,
            "normal_force": float(current_normal_force),
            "shear_force": float(shear_force),
            "slip": float(y_payload),
            "reflex_active": bool(state.reflex_active),
            "dropped": dropped
        })
        
    print(f"\n{C_BOLD}Trial Result Summary:{C_END}")
    print(f"Somatic Reflex Bridge: {'ENABLED' if apply_ztp else 'DISABLED'}")
    print(f"Final Grip Force: {current_normal_force:.2f} N")
    print(f"Final Slip Displacement: {y_payload*100:.2f} cm")
    print(f"Result: {C_GREEN}SUCCESS (Grasp maintained dynamically){C_END}" if not dropped else f"{C_RED}FAILURE (Payload dropped during VLA latency window){C_END}")
    
    if apply_ztp and not dropped:
        log_bytes = json.dumps(log).encode("utf-8")
        seal = hashlib.sha256(log_bytes).hexdigest()
        print(f"⚡ {C_BOLD}SHA-256 Somatic Telemetry Signature:{C_END} {C_BLUE}{seal}{C_END}")
        
    return not dropped

def main():
    print(BANNER)
    
    if not HAS_ZTP:
        print("Failed to run example due to missing FFI bindings.")
        sys.exit(1)
        
    # Trial 1: Low-frequency VLA commands the gripper BLINDLY (explodes / drops)
    simulate_vla_bridge(apply_ztp=False)
    
    print("\n" + "="*90 + "\n")
    
    # Trial 2: Low-frequency VLA bridged with 1000Hz Somatic FFI reflex (succeeds)
    simulate_vla_bridge(apply_ztp=True)

if __name__ == "__main__":
    main()
