# Verifikation – 7. November 2026

Historischer MVP-Stand. Der aktuelle SO-101-Nachweis steht in [SO101_REPORT.md](SO101_REPORT.md).

## Entwicklungsumgebung

Ubuntu 22.04.5 LTS, x86_64, Python 3.10.12. Kein Docker-CLI und kein Docker-Socket/
Daemon vorhanden; dies wurde zusätzlich außerhalb der Sandbox geprüft. Eine vorhandene
Host-ROS-Installation wurde weder benötigt noch verändert. Der Desktopstart entfernt
geerbte ROS-Pythonpfade nur für seinen eigenen Prozess.

## Abschließende Prüfungen

| Prüfung | Ergebnis |
| --- | --- |
| pytest außerhalb der Sandbox, inklusive echter Loopback-WebSocket-Verbindung | **42 bestanden, 3 übersprungen**, 3,00 s |
| Drei ROS-Integrationstests, ausdrücklich mit `--run-docker` angefordert | Übersprungen: Docker-Socket/Daemon fehlt |
| Ruff Lint | Bestanden |
| Ruff Formatprüfung | 43 Python-Dateien korrekt formatiert |
| `pip check` ohne Host-ROS-PYTHONPATH | Keine defekten Anforderungen |
| Shellsyntax von Bootstrap, Docker-Installer und Container-Entrypoint | Bestanden |
| `git diff --check` | Bestanden |
| Qt-Offscreen-Start | Erfolgreich, mit Offline-Signalen und Plot |
| Sichtbarer nativer X11-Start über `./start.sh` | Erfolgreich, anschließend kontrolliert beendet |
| Screenshots der endgültigen Haupt-, Monitoring- und Assistentenansicht | `docs/studio.png`, `docs/studio-monitoring.png`, `docs/setup.png` |

Abgedeckt sind Projekt-Roundtrip, invalides Format, Sensorgrenzen und TF-Offsets,
portable URDF-Kopien, URDF-Fehler und Vorwärtskinematik, sichere Plugin-Metadaten,
Katalog, gemockte Docker-Verfügbarkeit, Besitzprüfung und Ressourcenlimits,
Runtime-Health, Offline-GUI, Plot-Pause/Reset, Speichern/Öffnen durch die GUI,
Ersteinrichtung ohne Docker, Verbindungsfehler und Reconnect. Ein echter lokaler
WebSocket-Server prüft Graph-Protokoll, Nachrichten und erneutes Abonnieren.
Der laufende Container-Versionswechsel ist zusätzlich mit einem gemockten
RuntimeManager auf korrekte Stop/Start-Reihenfolge geprüft. Fehlerhafte URDF-/YAML-
Importe lassen das vorhandene Projekt bestehen.

In der eingeschränkten Sandbox wird zusätzlich der Loopback-Servertest ausgelassen,
wenn Socket-Bindings verboten sind. Die abschließende oben genannte Prüfung fand
mit erlaubten Sockets statt. Eine anfängliche PyQtGraph-Abschlusswarnung wurde durch
explizites Schließen des Plotwidgets behoben; der letzte Testlauf ist ohne diese Warnung.

## Live geprüfte Registry

Registry-v2-Manifeste für linux/amd64 wurden am 7. November 2026 direkt von Docker Hub
abgerufen. Rohdaten und Digests stehen in `registry-manifests.json`.

| Basisimage | Komprimierte Basis-Layer, dezimale MB |
| --- | ---: |
| `ros:humble-ros-base-jammy` | 264,17 |
| `ros:jazzy-ros-base-noble` | 296,62 |
| `ros:lyrical-ros-base-resolute` | 341,06 |

Diese Zahlen schließen Zusatzpakete, entpackten Speicher und Buildcache aus.
Die Laufzeitinstallation prüft erneut und pinnt den jeweils ermittelten Basisdigest.
Die [offiziellen ROS-Image-Tags](https://hub.docker.com/_/ros/tags) bestätigen die
gewählten Basisnamen. Der [ROS-Paketindex für rosbridge_suite](https://index.ros.org/p/rosbridge_suite/)
und der [xacro-Index](https://index.ros.org/p/xacro/) führen die drei Distributionen
als veröffentlichte Pakete. Das ersetzt keinen ausgeführten Image-Build.

Die zusätzliche direkte HTTPS-Abfrage der ROS-Binärpaketindizes unter
`packages.ros.org/ros2/ubuntu/dists/.../Packages.gz` scheiterte in dieser Umgebung mit
`CERTIFICATE_VERIFY_FAILED: Hostname mismatch`. Die TLS-Verifikation blieb aktiv.
Eine binäre Apt-Verfügbarkeitsprüfung wurde damit hier nicht erfolgreich abgeschlossen;
der Docker-Build enthält deshalb zwingend `apt-cache show` und bricht bei Fehlern ab.

## Nicht ausgeführte Prüfungen

- Keine tatsächlichen Builds der drei Studio-Erweiterungsimages.
- Keine echten Humble-/Jazzy-/Lyrical-Containerläufe; End-to-End-Abnahme B/E steht aus.
- Keine Ausführung des administrativen Docker-Installers, keine Änderung von Gruppen,
  Systempaketquellen oder bestehenden Containern.
- Keine echte Xacro-Ausführung im Container und keine physische Roboterhardware.

Diese offenen Integrationsprüfungen sind durch Unit-/Protokolltests nicht ersetzt.
Sie können mit den dokumentierten GUI-Verifikationsaktionen und opt-in Tests auf
einem Docker-fähigen Rechner nachgeholt werden.

## Weitere Primärquellen

- [Docker Engine auf Ubuntu](https://docs.docker.com/engine/install/ubuntu/): Grundlage der expliziten Installationsroutine.
- [rosbridge-Protokoll](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md): Topic- und Servicekommunikation.
- [rosbridge-Launchdatei](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/rosbridge_server/launch/rosbridge_websocket_launch.xml): Adapter und rosapi.
- [ROS-2-Lyrical-Veröffentlichung](https://discourse.openrobotics.org/t/ros-2-lyrical-luth-released/55021): Ubuntu 26.04 und Support bis Mai 2031.
