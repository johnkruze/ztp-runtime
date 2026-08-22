"""Load libztp_runtime for the Python ctypes examples.

Lives next to the demos. The kernel is two directories up:
  ztp-runtime/target/release/libztp_runtime.{dylib,so,dll}

Override: ZTP_RUNTIME_LIB=/path/to/libztp_runtime.dylib
"""
import os
import sys
import ctypes


def load_ztp_library(script_dir: str) -> ctypes.CDLL:
    env_path = os.environ.get("ZTP_RUNTIME_LIB")
    if env_path:
        if os.path.exists(env_path):
            return ctypes.CDLL(env_path)
        raise FileNotFoundError(f"ZTP_RUNTIME_LIB set but missing: {env_path}")

    if sys.platform.startswith("darwin"):
        lib_name = "libztp_runtime.dylib"
    elif sys.platform.startswith("win32"):
        lib_name = "ztp_runtime.dll"
    else:
        lib_name = "libztp_runtime.so"

    crate_root = os.path.abspath(os.path.join(script_dir, "..", ".."))
    candidates = [
        os.path.join(crate_root, "target", "release", lib_name),
        os.path.join(crate_root, "target", "debug", lib_name),
        # Spectrum eval box (sibling of this crate)
        os.path.join(crate_root, "..", "grokd", "public", "ztp-runtime-eval", "lib", lib_name),
        os.path.join(script_dir, lib_name),
    ]

    for path in candidates:
        normalized = os.path.normpath(path)
        if os.path.exists(normalized):
            return ctypes.CDLL(normalized)

    try:
        return ctypes.CDLL(lib_name)
    except OSError as e:
        raise FileNotFoundError(
            f"{lib_name} not found.\n"
            "Build the kernel:\n"
            "  cd ztp-runtime && cargo build --release\n"
            "Or set ZTP_RUNTIME_LIB to the compiled library."
        ) from e
