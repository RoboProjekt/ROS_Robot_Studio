# Entwicklung

## Umgebung

Ubuntu 22.04 x86_64, Python 3.10.12. Die direkten Python-Abhängigkeiten sind in
`pyproject.toml` gepinnt; `requirements.lock.txt` hält den getesteten vollständigen
Python-Auflösungsstand fest. Systembibliotheken und Container-Aptpakete sind separat.

```bash
./start.sh --prepare-only
./start.sh --demo
```

Der Bootstrap benötigt keine globale pip-Installation. Für Tests in einer bereits
mit ROS gesourcten Shell `PYTHONPATH` nur für den Testprozess entfernen:

```bash
env -u PYTHONPATH QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest -q
.venv/bin/ruff check src tests scripts
.venv/bin/ruff format --check src tests scripts
```

Ein lokaler WebSocket-Protokolltest verwendet ausschließlich Loopback-Sockets. In einer
Sandbox, die Socket-Bindings verbietet, wird er ausdrücklich ausgelassen; außerhalb der
Sandbox wurde er inklusive Reconnect und Wiederherstellung der Abonnements bestanden.
Docker-Unit-Tests verwenden Mocks.

## Echte ROS-Integration

Zunächst über die GUI das gewünschte Studio-Image installieren. Danach:

```bash
env -u PYTHONPATH .venv/bin/python -m pytest -q --run-docker -ra
```

Diese Tests laden keine großen Images automatisch herunter. Ohne Daemon oder ohne
installiertes Studio-Image wird der betreffende Test mit Grund übersprungen. Getestet
werden Humble, Jazzy und Lyrical parametrisiert: zwei echte Topics, numerische Nachricht,
rosapi-Topics, Node-Liste und empfangene Nachrichten im Subscriber-Log. Jeder Test erzeugt
seinen eigenen gelabelten Container auf einem freien Loopback-Port und entfernt ihn
in `finally`. Keine vorbestehenden Container werden angefasst.

## Docker manuell bauen

Bevorzugt den Installer in der GUI benutzen: Er prüft den amd64-Digest, zeigt
Download-/Buildfortschritt und versieht das Image mit dem benötigten Ownership-Label.
Für einen manuellen Humble-Build in einer eigenen Docker-Umgebung:

```bash
docker build --platform linux/amd64 \
  --build-arg BASE_IMAGE=ros:humble-ros-base-jammy \
  --build-arg DISTRO=humble \
  --label io.ros-robot-studio.managed=0.1.0 \
  -t ros-robot-studio/humble:0.1.0 src/ros_robot_studio/data/runtime
```

Für stärkere Reproduzierbarkeit BASE_IMAGE auf einen verifizierten Registry-Digest
setzen. Die GUI erledigt das automatisch. Jazzy verwendet `ros:jazzy-ros-base-noble`,
Lyrical `ros:lyrical-ros-base-resolute`. `DISTRO` und Tag entsprechend ändern.

Die Nur-Lese-Prüfung `env -u PYTHONPATH .venv/bin/python scripts/verify_catalog.py`
prüft Registry-Manifeste und versucht die offiziellen ROS-Binärpaketindizes zu lesen.
Sie schreibt `docs/registry-manifests.json`, zieht keine Images und installiert nichts.
Der Paketindex-TLS-Fehler der Entwicklungsumgebung wurde nicht durch Abschalten der
Zertifikatsprüfung umgangen.

## Debugging und visuelle Prüfung

- Strukturierte JSON-Logs: `${XDG_STATE_HOME:-~/.local/state}/ros-robot-studio/logs/studio.jsonl`.
- GUI-Containerlogs: **Project & Runtime → Fetch logs** (letzte 200 Zeilen).
- Headless-Start: `QT_QPA_PLATFORM=offscreen ./start.sh --demo --smoke-test`.
- Screenshots: `./start.sh --demo --screenshot docs/studio.png` erstellt zusätzlich
  `docs/studio-monitoring.png` und beendet die Anwendung anschließend.
- `--project examples/generic_robot/demo.studio.yaml` öffnet das Beispiel über den echten
  Projektloader. Änderungen müssen explizit gespeichert werden.

## Git

Das lokale Repository besitzt Branch `main`, keine Remotes und keine Push-Automatik.
Quellcode, kleine Ressourcen, Tests und Dokumentation werden versioniert. `.venv`,
Docker-Images, Logs, Caches, große Meshes/Modelle und Credentials sind ausgeschlossen.
Neue Funktionalität sollte Datenverträge, vorhandene Fehlerfälle und passende Tests
mitbringen. Bei einer neuen Distribution müssen Katalog, gültige Distribution-IDs,
Manifeste, Images und echte Integrationstests gemeinsam erweitert werden.


## Robotermodelle und SO-101 prüfen

```bash
# Alle lokalen Tests inklusive Modellinstaller-Mocks; keine Modelldownloads:
env -u PYTHONPATH QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest -q
# Expliziter Originaldownload in ein temporäres Testverzeichnis + selektive Reparatur:
env -u PYTHONPATH QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest -q --run-model-download -ra
```

Der SO-101-Downloadtest installiert keine Systemkomponenten und berührt keine echte
Hardware. `--run-docker` bleibt eine separate Option. Ohne freigegebene Loopback-Sockets
wird der WebSocket-Servertest übersprungen; die Abschlussprüfung lief mit erlaubten Sockets.

Nach Installation des normalen SO-101-Modells steht ein nativer sichtbarer Abnahmetest bereit:

```bash
env -u PYTHONPATH \
  LD_LIBRARY_PATH="$PWD/.venv/native/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  .venv/bin/python scripts/verify_so101_gui.py
```

Er prüft Auswahl, Gelenke, Pose, GL-Matrizen, Orbit/Pan/Zoom, Achsen/Meshumschaltung,
Speichern/Laden und schreibt zwei Screenshots sowie `docs/so101-gui-verification.json`.
Er setzt einen echten grafischen OpenGL-Kontext voraus und lädt selbst keine Modelle.
Für den normalen Benutzerstart genügt weiterhin `./start.sh`.

Der Maintainer-Helfer `scripts/pin_so101_model.py` erzeugt den kleinen Quell-Lock aus
einer ausdrücklich eingetragenen unveränderlichen Git-Revision. Er lädt nur URDF,
referenzierte STL-Dateien, Lizenz und Quell-README. Vor einer Katalogaktualisierung
Originalquelle, Lizenz und gemeinsame Kompatibilität von URDF und Meshes prüfen; keine
bestehende Projekt-Revision still ersetzen. Modellcache/Downloads bleiben außerhalb Git.

## Kinematik und Pick-and-Place entwickeln

SciPy bleibt optional und versionsgebunden:

```bash
env -u PYTHONPATH .venv/bin/python -m pip install -r requirements-kinematics.lock.txt
env -u PYTHONPATH QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest -q
.venv/bin/ruff check src tests scripts
.venv/bin/ruff format --check src tests scripts
./start.sh --pick-place-demo
```

`test_universal_kinematics.py` prüft neben dem bisherigen URDF-/FK-Regressionssatz
Position/Axis/Full Pose, mehrere Startwerte, Singularität, Konvergenzfehler, Grenzen und
unbekannte Frames. `test_motion.py` prüft Cubic-Peaks einschließlich Mimic, fehlende und
Null-Limits, Teach-in, Migration, Taskfehler und deterministische Würfelbindung.
`test_motion_gui.py` prüft echte Qt-Bedienelemente, Assistent, Bibliothek und Abbruch;
auch ein Anwendungsstart ohne optionale SciPy-Komponente wird geprüft.
SO-101-Tests lesen nur das bereits installierte geprüfte Originalmodell und werden ohne
Installation mit Grund ausgelassen. Generischer 3P-Arm und eigener 6R-Testarm sind originale
kleine Testressourcen. Der 6R-Test ist kein Nachweis für einen realen xArm 6.

Native Qt-/OpenGL-Abnahme mit installiertem SO-101:

```bash
env -u PYTHONPATH QT_QPA_PLATFORM=xcb \
  LD_LIBRARY_PATH="$PWD/.venv/native/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  .venv/bin/python scripts/verify_motion_gui.py
```

Das Skript verwendet ein temporäres Projekt und Settings, verändert keine Roboterhardware
oder fremden Container und schreibt `docs/kinematics.png`, `docs/pick-place-editor.png`,
`docs/pick-place-wizard.png` sowie `docs/motion-gui-verification.json`. Es prüft IK-Vorschau
und explizite Übernahme, Ablehnen eines unerreichbaren Ziels, alle sieben Wizardseiten,
Greifer/Kubus, Pause/Resume/Stop/Reset, Sortieren, Einzelberechnung und Save/Open/Play.

Paketierte direkt ladbare Projekte liegen unter `src/ros_robot_studio/data/examples/`:
`SO101_PickAndPlace_Demo.studio.yaml` und `Generic_PickAndPlace_Demo.studio.yaml`.
Der Menü-/CLI-Einstieg baut dieselben Beispieldaten über `motion.demos.pick_place_demo`
neu auf und validiert sie gegen die tatsächliche URDF. Modellbezogene Beispiel-Jointwerte
sind Teach-in-Daten; Solver und Taskgenerator enthalten keine SO-spezifischen Algorithmen.

## Robot State, ROS-Actions und echte Runtime-Abnahme

```bash
# Lokal / ohne Socket- und Dockerfreigabe:
env -u PYTHONPATH QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest -q
# Alle echten Runtime-/SO-101-Tasktests, nachdem die Images installiert wurden:
env -u PYTHONPATH QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest -q --run-docker --run-model-download -ra
# Einzelne reale End-to-End-Abnahme:
env -u PYTHONPATH QT_QPA_PLATFORM=offscreen .venv/bin/python scripts/verify_ros_runtime.py --distro humble
# Native Screenshots und GUI-Abnahme:
env -u PYTHONPATH QT_QPA_PLATFORM=xcb \
  LD_LIBRARY_PATH="$PWD/.venv/native/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  .venv/bin/python scripts/verify_ros_runtime.py --distro humble --screenshots
# Optionaler JointState-Kommandopfad, privater ROS-Szenenemulator (kein Isaac-Test):
env -u PYTHONPATH QT_QPA_PLATFORM=offscreen .venv/bin/python scripts/verify_external_adapter.py
```

`test_robot_state.py` prüft Leases, Zeit, Namen, fehlende/ungültige Daten, Threadreihenfolge,
FK und analytische Ableitungen. `test_ros_execution.py` prüft Cubic→ROS→Hermite-Gleichheit,
Start-/Velocitylimits, Goalannahme, Fehler, Timeout, Abbruch und Feedback als Erfolgskriterium.
`test_ros_integration.py` startet pro Distribution dieselbe echte GUI-/ROS-Teststrecke in
einem separaten Prozess mit temporären Settings und ausschließlich eigenen Containern.
Ein freier Loopback-Port wird gewählt. Ohne erforderliche Voraussetzungen wird mit Grund
übersprungen; es gibt keine impliziten großen Downloads aus den Tests.

Nach Änderungen an `data/runtime`, dem geteilten URDF-Parser, der Interpolation oder dem
Controllerkern ändert sich `runtime_source_hash`. Vor erneuter ROS-Ausführung müssen die
Images über Install Runtime aktualisiert werden. Die Paket-Layer bleiben im Docker-Buildcache.
Quellfingerabdruck und echte Image-ID sind von Verified-Diagnose getrennt.

Die neue Hauptansicht für Monitoring ist Robot State. Tests der alten Signalinspektion
wählen die synthetische Ansicht ausdrücklich; sie dürfen kein unabhängiges Robotersinus-
Playback als Standard erwarten. Das Projektformat bleibt v3 mit optionalen neuen Verträgen;
alte v1/v2/v3-Dateien laden weiterhin. Runtimezustände und aktive Leases werden nicht gespeichert.

## GUI-Lebensdauer und Qt-Version

Die Desktop-venv verwendet gemeinsam gepinntes PySide6/Addons/Essentials/Shiboken **6.9.3**,
PyQtGraph **0.13.7**, NumPy **2.2.6** und PyOpenGL **3.1.9**. Historische Berichte mit 6.8.3
beschreiben frühere Abnahmen. Aktueller Befund und Begründung:
[GUI_STABILITY_REPORT.md](docs/GUI_STABILITY_REPORT.md).

```bash
env -u PYTHONPATH .venv/bin/python -m pip check
env -u PYTHONPATH QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest -q \
  tests/test_gui_lifecycle.py --run-model-download
env -u PYTHONPATH QT_QPA_PLATFORM=xcb \
  LD_LIBRARY_PATH="$PWD/.venv/native/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  .venv/bin/python scripts/verify_gui_lifecycle.py --robot so101 --cycles 20 \
  --report docs/gui-lifecycle-verification.json
```

Der Prozess-Timeout ist an die Zykluszahl gekoppelt: 20 originale SO-101-Wizard-/IK-
Berechnungen können mit konkurrierenden Tests länger als 150 Sekunden dauern. Native
Prüfungen validieren Framebuffer und Shaderprogramme in jedem neu erstellten Kontext.
Die Tests lassen GC eingeschaltet, liefern nach Close noch gequeue-te Nachrichten aus,
prüfen `DeferredDelete`, joinen die Threadpools und verlangen keine verbleibenden Widgets.
