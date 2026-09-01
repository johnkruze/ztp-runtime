# C_FFI_BINDINGS

Authoritative material:

| File | Role |
|------|------|
| **`README.md`** | Layout, domains, build, FFI export index |
| **`src/lib.rs`** | Live ABI: `#[repr(C)]` structs + `extern "C"` functions |
| **`examples/python/`** | ctypes demos that load this dylib |
| **Spectrum `grokd/public/ztp-runtime-eval/`** | C eval box: `make help` is the clock map. Ten binaries, one dylib. |
