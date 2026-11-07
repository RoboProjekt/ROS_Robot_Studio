# Architektur

## Module und Abhängigkeiten

| Modul | Verantwortung |
| --- | --- |
| `app` | CLI, QApplication, rotierende strukturierte JSON-Logs |
| `gui` | Vier Qt-Tabs, Assistent, Dialoge, Status, Worker, Theme und Strukturviewer |
| `core` | Validierte Datenverträge und XDG-Einstellungen |
| `projects` | Atomisches YAML-Speichern, Laden und URDF-Assetkopien |
| `robots` | URDF-Baumvalidierung und Vorwärtskinematik mit NumPy |
| `ros_bridge` | Austauschbarer Signalvertrag, Offline-Transport, rosbridge-Client, Messpuffer |
| `runtime` | Docker SDK, Status, Registry-Katalog, Installation, Start/Stop, Healthchecks |
| `plugins` | Sichere deklarative Manifest-Discovery und Validierung |
| `compatibility` | Deklarative Kompatibilitätsberichte und zukünftiger MigrationRule-Vertrag |
| `data` | Paketierte Manifeste, Demo-URDF und distributionsübergreifender Docker-Buildkontext |

`sensors` reserviert einen Namespace für künftige Sensoradapter. Die heutigen
Sensor-Datenverträge liegen in `core.models`, die Konfigurationsoberfläche in `gui`.

## Kommunikation und Nebenläufigkeit

```text
Qt-Oberfläche ─── Project / RobotModel / PluginRegistry
      │
      ├── TaskRunner / QThreadPool ─── RuntimeManager ─── Docker SDK ─── Engine
      │                                                         │
      └── Transport-Signale                                      └── eigener ROS-Container
             ├── OfflineTransport / QTimer                            ├── rclpy Demo-Publisher
             └── RosbridgeTransport / QWebSocket ◀── 127.0.0.1 ────────┤
                                                                      ├── Demo-Subscriber
                                                                      └── rosbridge + rosapi
```

Docker-Operationen, Dateiladen/-speichern, Xacro-Verarbeitung und Netzprüfungen laufen
in Workern. Qt-WebSockets und Timer arbeiten ereignisgesteuert. Qt-Objekte werden im
GUI-Thread aktualisiert. Der RuntimeManager hat einen synchronen, ohne Qt testbaren
Vertrag; seine Methoden dürfen in der GUI nur über Worker aufgerufen werden.

Docker-Worker laufen seriell, damit Start, Stop und Versionswechsel sich nicht überholen.
Download-Progress wird auf maximal zehn Signale pro Sekunde begrenzt. Ein Disconnect
stoppt Reconnect-Timer, Graph-Timer und Abonnements, bevor der Transport ersetzt wird.

## Öffentliche Verträge

- `Project`, `Sensor`, `Transform`, `RuntimeConfig`: Pydantic-Modelle; zusätzliche und
  unbekannte Felder werden abgewiesen, Zahlen müssen endlich sein.
- `load_project(Path) -> Project`, `save_project(Project, Path) -> None`:
  Validierung vor atomischem Ersetzen der YAML-Datei.
- `parse_urdf(Path | str) -> RobotModel`, `RobotModel.positions(values) -> dict`:
  sichere XML-Verarbeitung und Gelenktransformationen in Metern/Radiant.
- `RuntimeManager.status() -> RuntimeStatus`; `install(distro, progress) -> image`;
  `start(distro, project_id, config) -> container_id`; `stop(id)`, `health(id)`, `logs(id)`.
  Nur eigene gelabelte Images/Container werden verändert. `stop` entfernt den Container,
  nie Projektdateien. Ein Client wird pro Operation geöffnet und anschließend geschlossen.
- `Transport`: `connect_transport`, `disconnect_transport`, `subscribe`, `unsubscribe`,
  `refresh_graph`, optional `topic_info`; Signale `status`, `graph`, `message`, `error`.
- `PluginRegistry`: YAML/JSON-Manifeste; keine dynamischen Imports oder Skript-Entrypoints.
- `check(project, target, registry) -> CompatibilityReport` und `MigrationRule` als
  zukünftiger Vertrag für überprüfbare Generierung in einem separaten Ausgabeordner.

Es wird kein `rclpy` im Desktop importiert. `start.sh` entfernt geerbtes ROS-PYTHONPATH
nur aus dem Anwendungsprozess. Bestehende ROS-Installationen werden nicht verändert.

## Runtime-Konzept

Offizielle Basisimages werden vor Installation über Registry v2 auf linux/amd64 geprüft
und mit dem ermittelten Digest gebaut. Der Build prüft rosbridge-/xacro-Pakete per
`apt-cache`, installiert aus den ROS-Paketquellen und protokolliert exakte Versionen in
`/opt/studio-packages.txt`. Basisdigest-Pinning und Python-Lock verbessern die
Reproduzierbarkeit; Apt-Abhängigkeiten sind noch kein vollständig eingefrorener Snapshot.

Jede Sitzung verwendet einen eigenen Container mit Ressourcenlimits, unprivilegiertem
Benutzer, verworfenen Linux-Capabilities und `no-new-privileges`. Kein Host-Netzwerk,
kein Docker-Socket-Mount, keine Host-Root-Mounts und keine USB-Freigabe. Der Containerport 9090 wird an `127.0.0.1` mit einem konfigurierbaren Hostport veröffentlicht.

Projektpersistenz liegt bewusst außerhalb der Container. Im MVP benötigt die Demo
keine Volumes. Normales Schließen stoppt den gestarteten eigenen Container asynchron.
Nach einem Prozessabsturz können eigene Container zurückbleiben; automatisches
Adoptieren oder eine vollständige Recovery-Oberfläche ist noch offen.

## Pufferung und Anzeige

rosbridge fordert 50 ms Throttling und Queue-Länge 1 an. Der Client behält pro Topic
nur die letzte Nachricht bis zum nächsten 50-ms-Flush. Empfangene WebSocket-Nachrichten
sind auf 1 MB begrenzt; Graph-Anfragen auf 32 offene Calls mit 10-s-Timeout. Reconnect
verwendet exponentielle Wartezeiten bis 15 Sekunden. Die UI rendert mit 1–30 Hz.

Der Plot speichert höchstens 6.000 Samples, erkennt bis zu 64 numerische Felder mit
begrenzter Verschachtelung und zeigt maximal acht Kurven. Bei 20 Hz entsprechen
6.000 Samples 300 Sekunden; niedrigere Nachrichtenraten verlängern die Pufferdauer.
Pause hält die aktuelle Plotansicht fest, während Verbindungsdiagnosen weiterlaufen.

## Plugin- und Migrationsgrenzen

Die Plugin-Verträge decken Roboter, Hardware, Controller, IK, Sensoren, Perception und
ROS-Adapter ab. Manifeste enthalten Version, ROS-Kompatibilität, Pakete, Abhängigkeiten,
Hardware, Interfaces, Parameter, Fähigkeiten und Implementierungsstatus. Unbekannte
oder inkompatible Komponenten erscheinen im Bericht. Geplante Pakete werden nicht
als getestet behauptet. Ein zukünftiger Code-Plugin-Host muss Prozesse isolieren und
Berechtigungen definieren; in diesem MVP wird kein fremder Plugin-Code ausgeführt.

Es gibt keine automatische ROS-Quellcodemigration. Ein Sitzungswechsel ändert weder
`default_ros` noch die ursprüngliche URDF. Distributionsspezifische Einstellungen sind
in `runtime.overrides` getrennt vorgesehen. Weitergehende Generierung ist noch offen.

Linux-Systemprüfungen und der optionale Ubuntu-Installer befinden sich im Runtime-Bereich.
Windows/macOS benötigen eigene Plattformadapter; GUI und Projektmodelle sind davon getrennt.
