# Zero-Trust Physics: C-FFI Developer Guide

This document details the **Foreign Function Interface (FFI)** bridging the high-performance **Rust physics core** (`ztp-runtime`) and the **Python cognitive loop** (`ztp_bridge.py`). 

By compiling the Rust engine as a dynamic library and loading it via Python's standard `ctypes` library, we achieve **near-zero execution overhead**, allowing high-frequency (1000Hz+) physics simulation to run seamlessly alongside local machine-learning vision models.

---

## 1. Architectural Flow

```mermaid
graph LR
    subgraph Python Space [Python Environment]
        Agent[Kid Cosmo VLM / Controller] -->|Sets Thrust / Inputs| Bridge[ztp_bridge.py]
    end
    
    subgraph FFI Boundary [ctypes FFI]
        Bridge -->|Passes Pointers & Structs| DLL[libztp_runtime.dylib]
    end
    
    subgraph Rust Space [Native Binary]
        DLL -->|Unsafe Pointer Deref| Solver[Rust Symplectic Solver]
        Solver -->|Calculates Forces & Integrations| Solver
        Solver -->|Writes Back to Shared Memory| DLL
    end
    
    DLL -.->|Returns Struct| Bridge
```

---

## 2. Memory Alignment & Data Structures

To prevent memory corruption or segment faults, all structures passed across the FFI boundary must have identical memory layout. In Rust, this is enforced using the `#[repr(C)]` attribute, which matches standard C-alignment. In Python, this is defined using `ctypes.Structure`.

### A. Mars EDL State & Results
Used for the Entry, Descent, and Landing simulation.

| Rust Struct (`#[repr(C)]`) | Python Class (`ctypes.Structure`) | Fields & Types |
| :--- | :--- | :--- |
| `C_MarsState` | `C_MarsState` | `position: [f64; 3]` (Coordinates)<br/>`velocity: [f64; 3]` (Speed)<br/>`dry_mass: f64` (Mass kg)<br/>`drag_area: f64` (Area m²)<br/>`cd: f64` (Drag coeff)<br/>`fuel_mass: f64` (Fuel kg)<br/>`specific_impulse: f64` (ISP s) |
| `C_MarsResult` | `C_MarsResult` | `density: f64` (Atmospheric density)<br/>`drag_force: [f64; 3]` (Newton drag)<br/>`net_accel: [f64; 3]` (Newton gravity + drag + thrust) |

#### Side-by-Side Code Comparison:
```rust
// Rust Definition (lib.rs)
#[repr(C)]
pub struct C_MarsState {
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub dry_mass: f64,
    pub drag_area: f64,
    pub cd: f64,
    pub fuel_mass: f64,
    pub specific_impulse: f64,
}
```
```python
# Python Definition (ztp_bridge.py)
class C_MarsState(ctypes.Structure):
    _fields_ = [
        ("position", ctypes.c_double * 3),
        ("velocity", ctypes.c_double * 3),
        ("dry_mass", ctypes.c_double),
        ("drag_area", ctypes.c_double),
        ("cd", ctypes.c_double),
        ("fuel_mass", ctypes.c_double),
        ("specific_impulse", ctypes.c_double),
    ]
```

---

### B. Satellite Orbit & Attitude State
Used for 6DoF orbit tracking and quaternion orientation controls.

```rust
// Rust Definition
#[repr(C)]
pub struct C_SatelliteState {
    pub position: [f64; 3],            // Orbit (km)
    pub velocity: [f64; 3],            // Speed (km/s)
    pub quaternion_attitude: [f64; 4], // Quaternion [w, x, y, z]
    pub angular_velocity: [f64; 3],    // Spin rate (rad/s)
    pub inertia_tensor: [f64; 9],      // 3x3 Flattened Matrix
}
```
```python
# Python Definition
class C_SatelliteState(ctypes.Structure):
    _fields_ = [
        ("position", ctypes.c_double * 3),
        ("velocity", ctypes.c_double * 3),
        ("quaternion_attitude", ctypes.c_double * 4),
        ("angular_velocity", ctypes.c_double * 3),
        ("inertia_tensor", ctypes.c_double * 9),
    ]
```

---

## 3. Function Declarations

Rust functions are exported as C-compatible symbols by disabling name mangling (`#[no_mangle]`) and declaring them as `pub extern "C"`.

### 1. Mars Integration (`ztp_mars_step`)
Advances the Mars lander integration.
* **Rust**:
  ```rust
  #[no_mangle]
  pub extern "C" fn ztp_mars_step(
      state: *mut C_MarsState, // Pointer to state struct
      retro_thrust: f64,       // Setpoint in Newtons
      dt: f64,                 // Timestep (e.g. 0.001s)
  ) -> C_MarsResult
  ```
* **Python ctypes configuration**:
  ```python
  _lib.ztp_mars_step.argtypes = [
      ctypes.POINTER(C_MarsState),
      ctypes.c_double,
      ctypes.c_double,
  ]
  _lib.ztp_mars_step.restype = C_MarsResult
  ```

---

### 2. Orbital 6DoF Translation (`ztp_orbital_step_6dof`)
Calculates J2-J4 planetary gravity perturbation translations.
* **Rust**:
  ```rust
  #[no_mangle]
  pub extern "C" fn ztp_orbital_step_6dof(
      state: *mut C_SatelliteState,
      dt: f64,
  )
  ```
* **Python ctypes configuration**:
  ```python
  _lib.ztp_orbital_step_6dof.argtypes = [
      ctypes.POINTER(C_SatelliteState),
      ctypes.c_double,
  ]
  _lib.ztp_orbital_step_6dof.restype = None
  ```

---

### 3. Orbital Attitude Control (`ztp_orbital_step_attitude`)
Integrates Euler's equations of rotational motion given external reaction wheel or thruster torques.
* **Rust**:
  ```rust
  #[no_mangle]
  pub extern "C" fn ztp_orbital_step_attitude(
      state: *mut C_SatelliteState,
      ext_torque_x: f64,
      ext_torque_y: f64,
      ext_torque_z: f64,
      dt: f64,
  )
  ```
* **Python ctypes configuration**:
  ```python
  _lib.ztp_orbital_step_attitude.argtypes = [
      ctypes.POINTER(C_SatelliteState),
      ctypes.c_double, # torque x
      ctypes.c_double, # torque y
      ctypes.c_double, # torque z
      ctypes.c_double, # dt
  ]
  _lib.ztp_orbital_step_attitude.restype = None
  ```

---

## 4. Compilation & Initialization

### Compilation (Rust)
To compile the dynamic library, run `cargo build` with the release profile. The output is configured in `Cargo.toml` as a `cdylib` (C-compatible dynamic library).

```bash
cd ztp-runtime
cargo build --release
```

**Output files by platform**:
* **macOS**: `target/release/libztp_runtime.dylib`
* **Linux**: `target/release/libztp_runtime.so`
* **Windows**: `target/release/ztp_runtime.dll`

### Loading (Python)
The library is loaded inside `ztp_bridge.py` using `ctypes.CDLL()`. It searches in relative directories and absolute paths to find the compiled binary:

```python
# Python Loading Example
import ctypes
import os

lib_path = os.path.abspath("ztp-runtime/target/release/libztp_runtime.dylib")
lib = ctypes.CDLL(lib_path)
```

---

## 5. Security & Pointer Safety

Because FFI boundaries utilize **raw pointers** (`*mut` and `*const` in Rust), the code executes in Rust's `unsafe` block. Developers must ensure the following safety protocols:

1. **Null Pointer Checks**: Rust functions actively verify `if state.is_null() { return; }` before dereferencing pointers to prevent segmentation faults.
2. **Flat Arrays**: Complex nested vectors (`Vec<T>`) cannot be passed directly across the C boundary. We use fixed-size flat float arrays (e.g. `[f64; 3]` or `[f64; 9]`) which translate directly to C-arrays.
3. **Rust Thread Safety**: The bridge functions are not thread-safe by default. Ensure calls to `_lib` methods are serialized or guarded if running inside concurrent python threads.
