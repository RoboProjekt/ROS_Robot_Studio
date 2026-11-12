# Architektur

## Module und Abhängigkeiten

| Modul | Verantwortung |
| --- | --- |
| `app` | CLI, QApplication, rotierende strukturierte JSON-Logs |
| `gui` | Fünf verschiebbare Qt-Panels, Assistent, Dialoge, Status, Worker, Theme und Strukturviewer |
| `core` | Validierte Datenverträge, unveränderliche RobotState-Snapshots, thread-sicherer State Manager, XDG-Einstellungen |
| `projects` | Atomisches YAML-Speichern, Laden und URDF-Assetkopien |
| `kinematics` | URDF-Ketten, Rang-/Konditionsanalyse, austauschbarer optionaler SciPy-IK-Solver |
| `motion` | Framed Teach-in, versionierte Tasks, Pick-and-Place, kubische Trajektorien, Offline-Backend |
| `robots` | Model Manager, Quell-Locks, STL-Import und gemeinsame homogene Vorwärtskinematik |
| `ros_bridge` | Austauschbarer Signalvertrag, Offline-Transport, rosbridge-Client, Messpuffer |
| `runtime` | Docker SDK, Status, Registry-Katalog, Installation, Start/Stop, Healthchecks |
| `vision` | Backendgebundene Kameraprofile, binärer RGB-Empfang, Inferenz-Worker und planare Lokalisierung |
| `plugins` | Sichere deklarative Manifest-Discovery und Validierung |
| `compatibility` | Deklarative Kompatibilitätsberichte und zukünftiger MigrationRule-Vertrag |
| `data` | Paketierte Manifeste, Demo-URDF und distributionsübergreifender Docker-Buildkontext |

`sensors` reserviert einen Namespace für künftige Sensoradapter. Die heutigen
Sensor-Datenverträge liegen in `core.models`, die Konfigurationsoberfläche in `gui`.

## RGB-Vision

Das Projekt speichert Kamera- und Ebenenprofile. Ein Kameraprofil verweist auf ein
Backendprofil und übernimmt dessen Domain. Ein ROS-Empfänger im vorhandenen
Backend-Container liefert RGB, CameraInfo und zeitgebundene TF2-Abfragen über einen
begrenzten Binärstream. Graph und Robotik verwenden weiterhin den rosbridge-Kanal.
Ein separater Worker führt Detektoren auf dem jüngsten RGB-Frame aus. Die Qt-Ansicht
verbindet Ergebnisse mit dem ausgewerteten Frame. Der VisionPoseProvider liefert
einen planaren Punkt und kann mit expliziten Werkzeugparametern einen Kinematikentwurf
erzeugen. Die SDK-interne Isaac-Komponente erstellt Camera Prims, Render Products und
ROS-Graphs außerhalb des Desktop-Prozesses.
[Transportvertrag](docs/ROS_CAMERA_TRANSPORT.md) · [Lokalisierung](docs/PLANAR_OBJECT_LOCALIZATION.md)

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
  `RobotModel.transforms` liefert alle 4×4-Linkposen; Visual-Origin und Skalierung werden
  separat verkettet. `geometry.load_robot` lädt lokale STL-Visuals ohne ROS.
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


## Robot Model Manager und SO-101 (8. November 2026)

`robots.model_manager.RobotModelManager` verwaltet überprüfte Modelldateien unabhängig
von ausführbaren Plugins. Der Datenkatalog unter `data/robot_models/` enthält IDs,
Versionen, Hersteller, Varianten, Lizenz, Quellen, Revision, Backendzustände und pro
Datei SHA-256/Größe. `plugin_id` verbindet ein Modell optional mit dem vorhandenen
Plugin-Metadatensystem. Neue Modelle verwenden denselben Installer, Parser und Viewer.

`status` prüft einen vollständigen registrierten Dateibaum; `install` lädt nur fehlende
oder beschädigte Dateien in ein temporäres Verzeichnis, validiert sämtliche Mesh-
Referenzen und aktiviert ihn erst danach. `resolve(model://...)` liefert ausschließlich
exakt überprüfte installierte Revisionen. Ein File-Lock serialisiert Installationen.
Alle diese I/O-Operationen laufen über den vorhandenen TaskRunner. Die XDG-Pfadverwaltung
liegt jetzt unabhängig in `core.paths`; `core.settings.xdg_path` bleibt kompatibel verfügbar.

Der zuvor eingeführte Projektformat-v2-Stand speichert portable Modellreferenzen, Modellversion/-revision,
Gelenkwerte, Endeffektor und vorbereitete Backendzustände. v1-Dateien werden im Speicher
angehoben. Eigene importierte URDFs behalten relative STL-Assets beim Speichern/Export;
Katalogmodelle werden aus dem zentralen XDG-Datenverzeichnis aufgelöst und bei Bedarf
exakt nachinstalliert. Große Modelle sind keine Git-/Projektdateien.

Die bestehende `RobotView` enthält zusätzlich eine PyQtGraph-OpenGL-Szene. Linkposen,
Visual-Offsets, Mesh-Skalierung und Materialfarben bleiben getrennt. trimesh lädt STL
ohne Reparatur oder Umwandlung; OpenGL aktualisiert beim Gelenkedit lediglich die
Instanzmatrizen. Qt erhält den bisherigen Software-Strukturviewer als ausdrücklich
beschrifteten Fallback ohne OpenGL. Kamerabedienung und Einpassen berücksichtigen die
tatsächlichen Visual-Bounds; angepasste Clip-Ebenen vermeiden Detail-Z-Fighting.

`gui.joint_controls.JointControls` erzeugt Regler aus echten URDF-Joints und speichert
SI-Werte. Fixed Joints sind nicht editierbar, mimic-Joints werden abgeleitet. Die
bestehende FK wurde um volle Linktransformationen, Material-/Visualdaten und mimic
vervollständigt; es gibt keine zweite Kinematik im Viewer. Generische Demo-Nachrichten
überschreiben keine manuelle Katalogroboterpose.

`JointMappingAdapter` ist ein reiner Vertrag für spätere kalibrierte Umrechnungen.
Die mitgelieferte unkalibrierte Implementierung verweigert jede Hardware-/ROS-Abbildung.
Der generische externe ROS-Simulationsadapter und die konkrete originale SO-101-Szene
sind inzwischen mit Isaac 5.1 geprüft. Die Gelenk-/ROS-Abnahme erteilt keine Hardwarefreigabe.
Es werden keine Motorbefehle erzeugt, keine LeRobot-/Isaac-Bibliotheken installiert und
keine Hardwaretreiber importiert.

## Universelle Kinematik und Bewegungsprogramme (Format v3)

`UniversalKinematicsEngine` nutzt `RobotModel.transforms` aus dem vorhandenen Parser.
Ketten entstehen aus Parent/Child-Pfaden; Fixed/Mimic/Continuous/Prismatic/Revolute,
Ursprünge, Achsen und dynamische Limits bleiben URDF-getrieben. Modell-Meshes sind optional.
Jacobi-Rangtests erkennen eingeschränkte Posekontrolle; Konditionsdiagnosen sind numerisch.

```text
MotionWorkspace / PickPlaceWizard / PositionManager
                  │
       KinematicChain / SolverManager ─── optional SciPy 1.15.3
                  │                         │
            TaskCompiler ◀──── validierte FK/IK-Endpunkte
                  │
        TrajectoryGenerator ─── Cubic + analytische Geschwindigkeits-/Beschleunigungspeaks
                  │
           CompiledTask ─── CollisionValidator (noch unavailable; hardware_ready=false)
                  │
           ExecutionBackend
                  ├── OfflinePreviewBackend ─── bestehender RobotView + virtueller Würfel
                  ├── ROS2ExecutionBackend ─── native rclpy-Actions im isolierten Container
                  └── IsaacSimProcessManager (externer SDK-Prozess) / Hardware: Read Only
```

Numerische Suchläufe und vollständige Taskvalidierungen laufen im eigenen seriellen
TaskRunner. Qt-Signale/Timer aktualisieren die Oberfläche; Stop und Kontextänderungen
brechen Suchen kooperativ ab und verhindern die Übernahme veralteter Ergebnisse.
Ein Robotertausch verwirft die berechnete Kette und Trajektorie. Original-URDFs werden
nicht verändert. `motion.schema` und `core.validation` vermeiden zyklische Datenmodellimporte.
SciPy wird erst im Solver geladen; die Basisanwendung bleibt ohne es startbar.

`SavedPosition` bindet Jointwerte per Modellsignatur, kartesische Ziele per explizitem
Frame/Tool. `MotionTask` enthält IDs, geordnete typisierte Schritte, Home, Bewegungsparameter
und optionales virtuelles Objekt. Manuelle/Saved-PoseProvider funktionieren; externe TF-/Vision-
Provider sind vorbereitet. Weitere Steuerstrukturen erfordern eine künftige Schema-/Interpreter-
Erweiterung. Eine fremde Bibliothek kann keine Hardwarefreigabe aus einem IK-Erfolg ableiten.

Ausführliche Konventionen, Algorithmusentscheidung und Grenzen: [KINEMATICS.md](docs/KINEMATICS.md).

## Gemeinsamer Robot State und echte ROS-Simulation

```text
Joint controls / explicit IK Apply / OfflinePreviewBackend
                           │
                           ▼
                  RobotStateManager ◀── Rosbridge JointStates
                           │                         ▲
                 Qt queued StateSignals             │
                           │                  ROS simulator
            ┌──────────────┼────────────┐            ▲
       Robot config   existing viewers  Monitoring   │
                                                   native ActionServer
TaskCompiler → existing JointTrajectory → ROS converter → container ActionClient
```

`RobotStateManager` ist die einzige Zustandsautorität. Modellinstanz und Quellen-Lease
verhindern alte Updates; Zeitstempel, Arrays, Namen, Mimic und Bounds werden geprüft.
Rendercaches sind keine getrennten Bewegungsquellen. Fehlende Messgrößen bleiben N/A.
Offline veröffentlicht analytische Cubic-Ableitungen. ROS-Tasks werden durch native
FollowJointTrajectory-Actions ausgeführt; beide Viewer folgen ausschließlich Rückmeldungen.

`ROS2ExecutionBackend` sequenziert denselben CompiledTask wie Offline. Das Gateway im
Container übersetzt begrenzte Session-/Goal-Kommandos in rclpy-Actions. Publish, Goalannahme,
Feedback, Result und beobachtete Zielwerte sind getrennte Zustände. Heartbeat, Timeouts,
Abbruch und Quellenwechsel verhindern unbeabsichtigte Fortsetzung. Hardware bleibt gesperrt.

Jede verwaltete Bridge-Sitzung hat ein eigenes Docker-Netz mit Ownership-Label; Stop
entfernt nur dieses Netz. Gleichnamige ROS-Topics paralleler Sitzungen vermischen sich nicht.

Der Runtime-Installer baut einen kuratierten kleinen Kontext aus gemeinsamem Parser,
Interpolation und Controllercode. Ein Quellfingerabdruck erkennt veraltete Images.
Verified verlangt tatsächlich ausgeführte native Action-/Feedbackdiagnose für die aktuelle
Image-ID. Alle drei Distributionen wurden real getestet; Apt-Builds sind keine vollständigen Snapshots.

Details: [Robot State Manager](docs/ROBOT_STATE_MANAGER.md), [ROS-Ausführung](docs/ROS_EXECUTION.md),
[Monitoring](docs/LIVE_MONITORING.md), [Runtime](docs/ROS_RUNTIME.md).
ROS- und Isaac-Simulation sind implementiert und mit tatsächlichen JointStates/Tasks geprüft.
Echte Hardware bleibt Read Only; eine kontaktphysikalische Griffvalidierung bleibt offen.

## Universal Backend Manager (Fortsetzung 8. November 2026)

Die bisherige Kinematik, Taskengine und State-Verwaltung werden über `backends/manager.py` und `gui/backend_workspace.py` an mehrere getrennte ROS-Sitzungen gebunden. Jede Sitzung besitzt Transport, Domainumgebung, Container, Port, Graph, Controllerstatus und Source-Lease. Der gemeinsame RobotStateManager enthält getrennte Backend-Kontexte; nur eine aktive Quelle projiziert nach Viewer/Monitoring. Der gemeinsame ROS2ExecutionBackend prüft zusätzlich aktive Backend-ID, Sitzungskennung und Modellsignatur.

`data/plugins/backend-*.yaml` erweitert den vorhandenen PluginRegistry deklarativ. Reale Hardware ist in Profil, Runtime und Transport schreibgeschützt. Isaac-SDK-Code liegt ausschließlich im extern gestarteten Szenenskript, nicht im Desktopprozess. Projektprofile sind portabel, installationsabhängige Pfade liegen lokal in Settings. [Ausführliche Architektur](docs/ROS_BACKEND_MANAGER.md), [DDS-Konfiguration](docs/ROS_MULTI_DOMAIN.md).

## Workspace und Runtime-Banner

`gui/workspace.py` hängt die vier vorhandenen Panels als QTabWidget-Gruppen in einen
QSplitter-Baum. `MainWindow.tabs` erhält die bisherige kanonische Panelindex-API. Der Baum
speichert ausschließlich UI-Identitäten und lokale Geometrie, keine State-/ROS-Kopien.
Eine Staging-Fläche hält Reparenting unter demselben Top-Level-Fenster. Layoutversion und
vollständige eindeutige Panelbelegung werden vor dem Wiederherstellen validiert.

`gui/simulation_header.py` liest und schreibt die zentrale Auswahl des bestehenden
ROSBackendManager. Lokale MRU-Dateien und Auto Play liegen in Settings; portable Projekte
enthalten weiterhin Profile mit logischen Referenzen. SceneManager und ProcessManager
bleiben die einzigen Isaac-Szenen-/Prozessverantwortlichen. Der SDK-Prozess pausiert nach
der Szenenprüfung; explizites Auto Play wird als separates Startargument übergeben.

`runtime/docker_diagnostics.py` löst den tatsächlichen Context/Endpoint auf und vergleicht
NSS-Kontogruppen mit den wirksamen Gruppen des aufrufenden Studio-Prozesses. RuntimeStatus
trägt denselben Snapshot zu GUI, Setup und Recheck. Die Diagnose verändert keine Rechte.
`TaskStep` erweitert vorhandene gespeicherte Ziele um optionale direkte Posen beziehungsweise
modellgebundene Jointwerte. Der bestehende TaskCompiler verarbeitet sie über dieselben
Positions-, IK-, Trajektorien- und Ausführungsverträge.

Details: [Docking](docs/GUI_DOCKING.md), [Runtime-Banner](docs/SIMULATION_RUNTIME_SELECTION.md),
[Taskeditor](docs/TASK_EDITOR_GUI.md), [Docker](docs/DOCKER_DIAGNOSTICS.md).
