# C choir — one dylib, many clocks

Hz is the body’s. Grasp is not hypha is not tesseract is not ocean.

| target | clock | dt | ethic |
|--------|-------|----|-------|
| `hold` | 1 kHz | 0.001 s | do not drop the part (45 N clamp) |
| `tissue` | 1 kHz | 0.001 s | do not destroy the sample (1.2 N liver) |
| `hypha` | 10 Hz | 0.1 s | Kirchhoff. Health conducts. |
| `tesseract` | host 1 kHz | 0.001 s | IMU firewall. Resonator ω_n is 100 Hz. |
| `ocean` | 1 kHz tick | 0.001 s | Mackenzie named sea. Not 1500. |
| `peek` | syscall | EOF | last-state. Magic SOMA. 64 B when the radio is dead. |

```
cd ztp-runtime
cargo build --release
cd examples/c && make check
```

Header: `include/ztp.h`. Law lives in `src/domains/`. Public `run:` is these six targets.

`hypha` writes `ztp-runtime/soma/mycelial_terminal.soma.bin` (body 8 peek). `./peek` generates a 64+64 fixture in memory, or takes a `.soma.bin` path. Tissue has no dedicated last-state body. Tesseract live orb is 8×f64 RAM; body 12 file is a named peek, not that binary.
