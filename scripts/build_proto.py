from pathlib import Path
import subprocess
import sys

NANOPB_ROOT = Path(".pio/libdeps/esp32dev/Nanopb")
PROTO_FILE = Path("protocol.proto")
C_OUTPUT_DIR = Path("firmware/src")
PY_OUTPUT_DIR = Path("cartpole/device")

subprocess.check_call([
    sys.executable, 
    NANOPB_ROOT / "generator" / "protoc",  
    "-I", NANOPB_ROOT / "generator" / "proto",
    "-I", PROTO_FILE.parent,
    "--nanopb_out", C_OUTPUT_DIR, 
    "--python_out", PY_OUTPUT_DIR, 
    PROTO_FILE
])
print("[build_proto.py] Protobuf bindings generated")
