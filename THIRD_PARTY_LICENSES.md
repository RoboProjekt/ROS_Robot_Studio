# Third-party components

Studio source and the original generic example: Apache-2.0 (see LICENSE).
Dependencies are not relicensed by Studio. Their distributed notices remain authoritative.

| Component | Version used | Upstream license / notice |
| --- | --- | --- |
| PySide6, Shiboken6, bundled Qt | 6.9.3 | LGPL-3.0 / GPL-3.0 / commercial options, module dependent. This application uses the open-source distribution. See [Qt for Python licensing](https://doc.qt.io/qtforpython-6/licenses.html) and bundled package notices. |
| PyQtGraph | 0.13.7 | MIT |
| trimesh | 4.6.8 | MIT |
| PyOpenGL | 3.1.9 | BSD-style license; original package notices apply |
| filelock | 3.18.0 | Unlicense |
| SO-101 follower URDF/STLs (downloaded separately) | Revision a758567c3978dfeefe282ede0500085a48fe8f78 | Apache-2.0; original LICENSE and source README retained beside model |
| SciPy (optional kinematics) | 1.15.3 | BSD-3-Clause; bundled OpenBLAS and other notices in the wheel retain their licenses |
| NumPy | 2.2.6 | BSD-3-Clause; bundled numerical-library notices also apply |
| Pydantic | 2.10.6 | MIT |
| PyYAML | 6.0.2 | MIT |
| defusedxml | 0.7.1 | PSF-2.0 |
| Docker SDK for Python | 7.1.0 | Apache-2.0 |
| pytest / pytest-qt | 8.3.5 / 4.4.0 | MIT |
| Ruff | 0.11.13 | MIT |
| websocket-client (test only) | 1.8.0 | Apache-2.0 |
| pip (bootstrap only) | 25.0.1 | MIT |
| libxcb-cursor0 (local fallback only) | Ubuntu 0.1.1-4ubuntu1 | MIT/X11; original copyright file retained in `.venv/native/usr/share/doc/libxcb-cursor0/copyright` |
| Official ROS Docker bases | Humble, Jazzy, Lyrical | Each image includes Ubuntu/ROS components under their respective licenses |
| rosbridge_suite / xacro (container only) | Distribution packages | BSD; see ROS Index and package notices inside image |

The complete resolved Python dependency list is `requirements.lock.txt`; transitive
packages retain their individual licenses in `.venv/lib/python3.10/site-packages/*dist-info/`.
No external robot meshes, model weights, icons or proprietary assets are committed to Git.
The SO-101 installer retrieves the reviewed original model under its upstream license;
see [source and notices](docs/SO101.md).
Text symbols are rendered using system fonts. Distribution of Qt binaries requires
preserving notices and complying with the relevant LGPL/GPL terms.
