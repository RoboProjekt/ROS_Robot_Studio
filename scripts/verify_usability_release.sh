#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
unset PYTHONPATH
export PYTHONNOUSERSITE=1
export LD_LIBRARY_PATH="$PWD/.venv/native/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
mkdir -p .bootstrap/usability-release
.venv/bin/python - <<'PY'
import json
from pathlib import Path
from ros_robot_studio.runtime.manager import RuntimeManager
status = RuntimeManager().status()
Path('.bootstrap/usability-release/docker-preflight.json').write_text(json.dumps(status.diagnostics, indent=2))
if not status.available:
    raise SystemExit(status.message)
missing = {'humble', 'jazzy', 'lyrical'} - set(status.installed)
if missing:
    raise SystemExit('Current owned runtime images missing: ' + ', '.join(sorted(missing)))
print('Docker preflight passed in the effective test-process session', flush=True)
PY
QT_QPA_PLATFORM=offscreen .venv/bin/pytest -q -o faulthandler_timeout=180 --run-docker --run-model-download --run-isaac \
  --junitxml=.bootstrap/usability-release/regression.xml 2>&1 | tee .bootstrap/usability-release/regression.log
QT_QPA_PLATFORM=xcb .venv/bin/python scripts/verify_gui_usability.py --cycles 8 --output docs \
  2>&1 | tee .bootstrap/usability-release/native.log
.venv/bin/ruff check src tests scripts
.venv/bin/ruff format --check src tests scripts
.venv/bin/python -m pip check
git diff --check
