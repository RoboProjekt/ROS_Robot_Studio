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
| OpenCV Contrib Python Headless (optional vision) | 4.12.0.88 | Apache-2.0 for OpenCV; the wheel includes additional third-party notices. [OpenCV license](https://github.com/opencv/opencv/blob/4.12.0/LICENSE) |
| ONNX Runtime (optional vision) | 1.22.1 | MIT; package notices apply. [ONNX Runtime license](https://github.com/microsoft/onnxruntime/blob/v1.22.1/LICENSE) |
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

The baseline Python lock is `requirements.lock.txt`. Optional dependencies are in
`requirements-kinematics.lock.txt` and `pyproject.toml`. Python distributions retain their
individual licenses under `.venv/lib/python3.10/site-packages/*dist-info/`.
The SO-101 installer retrieves the reviewed original model under its upstream license;
see [source and notices](docs/SO101.md).
The screenshot series uses the SO-101 geometry rendered in Studio and Isaac Sim with
procedural scene geometry. Screenshots illustrate the respective applications. The Isaac
Sim SDK, simulator binary assets, SO-101 mesh files and generated USD files are separate
local dependencies. [NVIDIA Isaac Sim distribution](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html)

The optional YOLO verification used Ultralytics YOLOv5n v7.0 under its upstream GPL-3.0
release terms. Model weights and the upstream reference photograph are external test
inputs. The adapter source is part of Studio. [Upstream model release](https://github.com/ultralytics/yolov5/releases/tag/v7.0) · [Upstream license](https://github.com/ultralytics/yolov5/blob/v7.0/LICENSE)
Text symbols are rendered using system fonts. Distribution of Qt binaries requires
preserving notices and complying with the relevant LGPL/GPL terms.
