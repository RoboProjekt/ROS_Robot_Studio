"""Supply Ubuntu 22.04's tiny optional Qt cursor dependency locally, without root."""
import ctypes.util
import hashlib
import os
from pathlib import Path
import platform
import subprocess
import sys
import urllib.request

root = Path(__file__).resolve().parents[1]
native = root / '.venv' / 'native'
library = native / 'usr/lib/x86_64-linux-gnu/libxcb-cursor.so.0'
if sys.platform != 'linux' or ctypes.util.find_library('xcb-cursor') or library.exists():
    raise SystemExit(0)
if os.environ.get('QT_QPA_PLATFORM') in ('offscreen', 'minimal'):
    raise SystemExit(0)
os_release = Path('/etc/os-release').read_text()
if 'VERSION_ID="22.04"' not in os_release or platform.machine() != 'x86_64':
    raise SystemExit('Qt requires libxcb-cursor0. Install it using your OS package manager, then retry.')
url = 'https://archive.ubuntu.com/ubuntu/pool/universe/x/xcb-util-cursor/libxcb-cursor0_0.1.1-4ubuntu1_amd64.deb'
expected = 'c9b5d1ad4af57397b1bd77e0a92750e34419def134c0282a0836ae9efc07cf64'
print('Preparing local Qt cursor library (10.5 kB; no system installation)…', flush=True)
with urllib.request.urlopen(url, timeout=30) as response:
    data = response.read()
if hashlib.sha256(data).hexdigest() != expected:
    raise SystemExit('Qt cursor package checksum mismatch')
package = root / '.bootstrap' / 'libxcb-cursor0.deb'
package.parent.mkdir(exist_ok=True)
package.write_bytes(data)
subprocess.check_call(['dpkg-deb', '--extract', str(package), str(native)])
