#!/usr/bin/env bash
set -euo pipefail
# Keep inherited host ROS Python packages out of the desktop interpreter.
unset PYTHONPATH
export PYTHONNOUSERSITE=1
export PIP_CACHE_DIR="$(dirname -- "${BASH_SOURCE[0]}")/.bootstrap/pip-cache"
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ ! -x .venv/bin/python ]]; then
  echo 'Creating local Python environment…'
  python3 -m venv --without-pip .venv
fi
if ! .venv/bin/python -m pip --version >/dev/null 2>&1; then
  .venv/bin/python scripts/bootstrap.py
fi
if [[ ! -f .venv/.studio-ready || pyproject.toml -nt .venv/.studio-ready ]]; then
  .venv/bin/python -m pip install -e '.[dev]'
  touch .venv/.studio-ready
fi
.venv/bin/python scripts/qt_native.py
export LD_LIBRARY_PATH="$PWD/.venv/native/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
if [[ "${1:-}" == '--prepare-only' ]]; then exit 0; fi
exec .venv/bin/python -m ros_robot_studio "$@"
