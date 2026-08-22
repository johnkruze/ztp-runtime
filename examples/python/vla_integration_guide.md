# VLA Somatic Bridge — Integration Guide

## The Gap This Closes

VLA policies (typical 5–50Hz vision-language-action stacks) run at 5–50Hz. Between inference calls, physics doesn't pause. A grip slip starts in 2ms. An oily surface patch changes the friction coefficient mid-grasp. The VLA won't see it until the next call — 20–200ms later. By then the object is already moving.

The somatic bridge runs at 1000Hz between your VLA calls. It evaluates every force command against physical invariants — friction cones, slip velocity, contact margin — and issues corrections before the disturbance propagates. Your VLA continues planning at its natural cadence. The somatic layer handles what happens in the spaces between.

**`vla_somatic_bridge.py`** demonstrates this directly: a VLA commanding a constant 12.0N drops its payload at t=0.468s. The same scenario with the bridge enabled catches micro-slip at t=0.016s, escalates to 22.68N, and holds the grasp through a friction coefficient collapse from 0.45 to 0.22.

---

## Prerequisites

```bash
# From this crate root
cargo build --release
#   target/release/libztp_runtime.dylib  (macOS)
#   target/release/libztp_runtime.so     (Linux)
```

The loader handles path resolution across platforms:

```python
from ztp_loader import load_ztp_library
lib = load_ztp_library(script_dir)
```

---

## The Interface

Three structures cross the FFI boundary. They must match the Rust definitions in `ztp-runtime/src/domains/dexterous.rs` exactly.

```python
import ctypes

class C_Taxel(ctypes.Structure):
    _fields_ = [
        ("normal",  ctypes.c_float),   # Normal force at this taxel (N)
        ("shear_x", ctypes.c_float),   # Tangential shear — horizontal axis (N)
        ("shear_y", ctypes.c_float),   # Tangential shear — vertical axis (N)
    ]

class C_TactileArray(ctypes.Structure):
    _fields_ = [("taxels", C_Taxel * 16)]  # 4×4 sensor matrix, row-major

class C_GraspState(ctypes.Structure):
    _fields_ = [
        ("normal_force",           ctypes.c_float),  # Current grip force (N) — seed from VLA command
        ("slip_velocity",          ctypes.c_float),  # Linear slip velocity (m/s) — read from IMU/encoder
        ("slip_angular_velocity",  ctypes.c_float),  # Rotational slip (rad/s)
        ("object_mass",            ctypes.c_float),  # Payload mass (kg)
        ("static_friction_coeff",  ctypes.c_float),  # Estimated static μ — bridge adapts this
        ("dynamic_friction_coeff", ctypes.c_float),  # Estimated dynamic μ
        ("reflex_active",          ctypes.c_bool),   # Whether reflex fired on the last step
    ]
```

The bridge returns:

```python
class C_GraspResult(ctypes.Structure):
    _fields_ = [
        ("micro_slip_detected",      ctypes.c_bool),   # Boundary taxels exceeding friction limit
        ("macro_slip_detected",      ctypes.c_bool),   # Global linear slip underway
        ("rotational_slip_detected", ctypes.c_bool),   # Torsional moment at contact boundary
        ("commanded_force",          ctypes.c_float),  # Somatic-corrected grip force — send this to actuator
        ("margin",                   ctypes.c_float),  # Friction cone safety margin (1.0=nominal, 0.0=slip boundary)
        ("estimated_mu",             ctypes.c_float),  # Running friction coefficient estimate
    ]
```

Wire the FFI signature:

```python
lib.ztp_dexterous_evaluate_grasp.argtypes = [
    ctypes.POINTER(C_TactileArray),
    ctypes.POINTER(C_GraspState),
    ctypes.c_float  # dt in seconds
]
lib.ztp_dexterous_evaluate_grasp.restype = C_GraspResult
```

---

## Wrapping Your VLA — The Integration Pattern

```python
dt = 0.001        # 1ms somatic integration step
vla_period = 0.2  # 5Hz VLA → 200ms between policy calls
steps_per_vla = int(vla_period / dt)  # 200 somatic steps per VLA inference

# Initialize somatic state from your sensors
state = C_GraspState()
state.object_mass            = measured_payload_mass    # kg
state.static_friction_coeff  = 0.5                     # starting estimate — bridge adapts it
state.dynamic_friction_coeff = 0.4
state.slip_velocity          = 0.0
state.slip_angular_velocity  = 0.0
state.reflex_active          = False

sensor = C_TactileArray()
somatic_log = []

while task_running:

    # ── VLA inference (your policy) ──────────────────────────────
    vla_force = your_vla_policy.step(image, language_instruction)
    state.normal_force = vla_force

    # ── 1000Hz somatic loop between VLA calls ────────────────────
    for _ in range(steps_per_vla):

        # Read your physical tactile sensor (16 taxels, normal + shear)
        populate_sensor_array(sensor, your_tactile_hardware.read())

        # Somatic evaluation at 1000Hz
        result = lib.ztp_dexterous_evaluate_grasp(
            ctypes.byref(sensor), ctypes.byref(state), dt
        )

        # Send the somatic-corrected force — not the raw VLA command
        your_gripper.set_force(result.commanded_force)

        # Feed corrections back into state for next step
        state.normal_force  = result.commanded_force
        state.slip_velocity = your_encoder.read_slip_velocity()

        # Collect somatic record
        somatic_log.append({
            "t":                current_time(),
            "vla_commanded":    vla_force,
            "somatic_force":    result.commanded_force,
            "margin":           result.margin,
            "estimated_mu":     result.estimated_mu,
            "micro_slip":       result.micro_slip_detected,
            "macro_slip":       result.macro_slip_detected,
        })
```

`populate_sensor_array` and `your_encoder.read_slip_velocity()` are your hardware interface points — fill in from your tactile sensor driver and actuator feedback.

---

## What Each Output Means Physically

**`commanded_force`** — use this, not the VLA output. When the friction margin is healthy, it equals what your policy commanded. When the margin collapses, the bridge projects the command up to re-establish the friction cone. The VLA keeps planning; the somatic layer keeps the hand attached to the object.

**`margin`** — friction cone safety margin. At 1.0, contact is nominal. At 0.0, you are at the slip boundary. The reflex fires below 0.1. This is a dense physical signal available at every 1ms step — your policy can learn to anticipate margin collapse rather than react to it.

**`estimated_mu`** — running friction coefficient estimate, updated from the tangential-to-normal force ratio at boundary taxels using a low-pass filter (α = 0.05). This is how the bridge adapts to surface transitions — wet floor, oil patch, worn rubber — without explicit friction measurement hardware.

**`micro_slip_detected`** — boundary taxels have exceeded the local friction limit. Slip is local and the reflex can still contain it. This is the signal that should trigger a VLA re-plan at the next inference cycle if it persists.

**`macro_slip_detected`** — global linear slip is underway. If this is true despite the reflex firing, the friction coefficient has dropped below what current grip force can recover at this payload mass.

---

## The Somatic Log as Training Data

Every row in `somatic_log` is labeled training data that standard demonstration datasets don't contain. Demonstrations capture the success case. The somatic log captures the physical near-miss and the correction that prevented the drop:

```python
# t=0.016s — the moment the demo dataset is silent
{
    "t":             0.016,
    "vla_commanded": 12.0,      # what the policy wanted
    "somatic_force": 22.68,     # what physics required
    "margin":        0.03,      # how close to the slip boundary
    "estimated_mu":  0.22,      # adapted friction estimate after surface change
    "micro_slip":    True,      # precursor — containable
    "macro_slip":    False      # reflex held it
}
```

This row teaches the policy: *when margin < 0.1 and estimated_mu < 0.3, the physically correct force is 22.68N, not 12.0N.* Fine-tuning on these rows teaches preemptive grip increase before the slip begins — the behavior that expert humans have but demos never capture because experts don't drop things.

Seal and export the run:

```python
import hashlib, json, pandas as pd

log_bytes = json.dumps(somatic_log).encode("utf-8")
somatic_seal = hashlib.sha256(log_bytes).hexdigest()

df = pd.DataFrame(somatic_log)
df["somatic_seal"] = somatic_seal
df.to_parquet(f"somatic_run_{int(current_time())}.parquet", index=False)
```

The signature is self-certifying — the run's attestation that every force correction was derived from physical invariants, step by step, without requiring an external authority to validate it.

---

## Scenario Coverage

These examples call the dylib. Sealed failure banks live in genesis-core / doe-genesis, not here.

| Example | FFI | Sealed sibling |
|---------|-----|----------------|
| `vla_somatic_bridge.py` | `ztp_dexterous_evaluate_grasp` | Topic 4 grasp bank |
| `dexterous_grasp.py` | same | same |
| `surgical_micro_test.py` | surgical + micro | grokd surgical / micro Parquets |
| `directed_energy.py` | `ztp_directed_energy_step` | directed-energy HF receipt |
| `biological_compounding_test.py` | compounding / vagal / seal | Topic 1 compounding bank |

---

## Run the Demo

```bash
cd ztp-runtime
cargo build --release
cd examples/python

python3 vla_somatic_bridge.py
python3 dexterous_grasp.py
python3 surgical_micro_test.py
```

C examples (hold / tissue / machine) live in the Spectrum eval box: `grokd/public/ztp-runtime-eval/`.

---

[ztp-runtime](https://github.com/johnkruze/ztp-runtime) — this crate
[genesis-core](https://github.com/johnkruze/genesis-core) — Forge
[datasets](https://huggingface.co/spiderpilot89) — receipts, MIT licensed

John Kruze · [ZeroTrustPhysics.com](https://ZeroTrustPhysics.com) · kruze@zerotrustphysics.com
