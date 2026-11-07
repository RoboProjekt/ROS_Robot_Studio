"""Install a fixed pip wheel into the project venv, without system changes."""

import hashlib
import json
import subprocess
import sys
import urllib.request
from pathlib import Path

root = Path(__file__).resolve().parents[1]
cache = root / ".bootstrap"
cache.mkdir(exist_ok=True)
with urllib.request.urlopen("https://pypi.org/pypi/pip/25.0.1/json", timeout=30) as response:
    metadata = json.load(response)
wheel = next(item for item in metadata["urls"] if item["filename"].endswith("py3-none-any.whl"))
with urllib.request.urlopen(wheel["url"], timeout=60) as response:
    data = response.read()
if hashlib.sha256(data).hexdigest() != wheel["digests"]["sha256"]:
    raise SystemExit("pip wheel checksum mismatch")
path = cache / wheel["filename"]
path.write_bytes(data)
# A wheel is a ZIP archive. Import only this verified pip wheel in the isolated interpreter.
subprocess.check_call(
    [
        sys.executable,
        "-c",
        'import sys,runpy; sys.path.insert(0,sys.argv.pop(1)); runpy.run_module("pip",run_name="__main__")',
        str(path),
        "install",
        "--no-deps",
        str(path),
    ]
)
