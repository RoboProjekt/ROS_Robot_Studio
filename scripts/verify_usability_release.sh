#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
unset PYTHONPATH
export PYTHONNOUSERSITE=1
export LD_LIBRARY_PATH="$PWD/.venv/native/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
mkdir -p .bootstrap/usability-release
.venv/bin/python - <<'PY'
import hashlib
import json
import subprocess
from pathlib import Path
files = [p for root in ('src','tests','scripts') for p in Path(root).rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix != '.pyc']
files += [Path(p) for p in ('pyproject.toml','requirements.lock.txt','requirements-kinematics.lock.txt','start.sh')]
manifest = dict(commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(), files={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(set(files))})
Path('.bootstrap/usability-release/source-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print('Frozen source commit:', manifest['commit'], flush=True)
PY
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
QT_QPA_PLATFORM=xcb .venv/bin/python scripts/verify_gui_lifecycle.py --robot so101 --cycles 20 \
  --report docs/gui-lifecycle-usability-verification.json 2>&1 | tee .bootstrap/usability-release/lifecycle.log
.venv/bin/python - <<'PY'
import hashlib
import json
from pathlib import Path
manifest = json.loads(Path('.bootstrap/usability-release/source-manifest.json').read_text())
changed = [p for p,digest in manifest['files'].items() if not Path(p).is_file() or hashlib.sha256(Path(p).read_bytes()).hexdigest()!=digest]
current = {str(p) for root in ('src','tests','scripts') for p in Path(root).rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix != '.pyc'}
changed += sorted(current - set(manifest['files']))
if changed:
    raise SystemExit('Source changed during acceptance: '+', '.join(changed))
print('Frozen source manifest unchanged throughout acceptance',flush=True)
PY
.venv/bin/ruff check src tests scripts
.venv/bin/ruff format --check src tests scripts
.venv/bin/python -m pip check
git diff --check
