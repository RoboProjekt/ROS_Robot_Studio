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
