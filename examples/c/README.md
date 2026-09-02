# hold — 12 N policy. 1000 Hz reflex. 45 N clamp.

Classical friction cone on the metal. Not taxel-timeseries data. The loop.

```
cd ztp-runtime
cargo build --release
cd examples/c && make && ./hold
```

Law: `src/domains/dexterous.rs` `evaluate_grasp_dynamics` · export `ztp_dexterous_evaluate_grasp`. Clock 1 kHz. Public header: `include/ztp.h`.

`./hold` starts at 12 N, μ 0.22, 0.80 kg, hostile shear. Reflex ramps force; clamp 45 N. Expected class: F leaves 12 N, F ≤ 45 N.
