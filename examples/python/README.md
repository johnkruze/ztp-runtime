# Python examples — ctypes on this dylib

These are **skin on `libztp_runtime`**. They are not a second product and not 13 domains.

C stranger path (hold / tissue / machine): `../../grokd/public/ztp-runtime-eval/` from Spectrum, or this crate’s public eval box sibling.

```bash
cd ../..   # ztp-runtime crate root
cargo build --release
cd examples/python
python3 vla_somatic_bridge.py          # 12 N drops; reflex holds at 16 ms
python3 dexterous_grasp.py             # tactile sweep
python3 surgical_micro_test.py         # 1.2 N liver + micro release
python3 directed_energy.py             # gimbal FFI
python3 biological_compounding_test.py # FKPP / Ostwald / Noyes / vagal / seal
```

`ztp_loader.py` finds `target/release/libztp_runtime.*`. Override with `ZTP_RUNTIME_LIB`.
