> **Historische Übergabe, fortgesetzt am 8. November 2026.** Der spätere ausdrückliche Fortsetzungsauftrag hebt die unten dokumentierte Pause auf. WIP `0e93253` wurde beibehalten. Die neue Implementierung beginnt mit `55bf1b0` (Backend-Sitzungen) und `979973f` (GUI/Quellenwechsel). Die finale Code-Abnahme umfasst 185 bestandene Tests ohne Fehler oder übersprungene Fälle sowie separate Stress-/GUI-Läufe. Der aktuelle Stand und offene Grenzen stehen in [BACKEND_VERIFICATION.md](BACKEND_VERIFICATION.md). Die nachfolgenden Abschnitte dokumentieren den früheren Pausenstand und sind keine aktuelle Funktionsbewertung.

# Entwicklungsübergabe: sichere Pause vor dem Codex-Update

Stand: **8. November 2026**. Repository:
`$HOME/Desktop/ROS_APP/ros-robot-studio`.

**Die Entwicklung ist auf ausdrücklichen Benutzerwunsch pausiert. Nicht automatisch
weiterarbeiten. Erst ein separater Fortsetzungsprompt hebt die Pause auf.**
Codex wird vom Benutzer außerhalb dieser Sitzung aktualisiert. An seiner Installation
wurde nichts geändert. Keine Änderungen verwerfen, kein Reset, keine History-Umschreibung,
kein Remote-Push. Der lokale Zwischenstand wird mit diesem Dokument unter dem Commit-Betreff
**`WIP: Multi-domain backend integration before Codex update`** gesichert; Hash über `git log`.

## 1. Auftrag und stabiler Ausgangspunkt

Aktiver, noch **nicht abgeschlossener** Entwicklungsauftrag:
**Universal ROS Backend Manager, Multi-Domain Architecture & Isaac Sim Integration**.
Der ursprüngliche vollständige Benutzerauftrag liegt auch unter:
`$HOME/.codex/attachments/b4a5f333-f9f1-4092-9228-557bc9e0e9c2/Pasted text.txt`.
Der Pausenauftrag:
`$HOME/.codex/attachments/6c16b658-a0ee-44d3-a71f-6a41fdd7565a/Pasted text.txt`.
Dieses Dokument hält den technischen Arbeitskontext ohne Chatverlauf fest.

Gewünschtes Ergebnis:

- Ein universeller `ROSBackendManager` verwaltet mehrere gleichzeitige Verbindungen.
  Pro Profil eigene ID, frei editierbare Domain, ROS-Distribution, Middleware, Endpunkt,
  Subscriptions, Graph und Controller. Die GUI bleibt ohne `rclpy` und ohne Isaac-API-Imports.
- Pro ROS-Verbindung ein vor Initialisierung konfigurierter Container/ROS-Kontext;
  Domainänderung erfordert dessen Neustart. Kein Ändern globaler Domain-Umgebungsvariablen
  im laufenden Desktop, keine automatische Weiterleitung zwischen Domains.
- Ein aktives Backend pro ausgewählter Robotermodellinstanz; andere Backends können
  getrennt beobachtet werden. Keine Vervielfältigung von Bewegungsbefehlen.
- Dieselben URDF-/FK-/IK-/Teach-in-/Task-/Trajektorienkomponenten wiederverwenden.
  Kein zweiter Planner und kein zusätzlicher RobotStateManager pro Backend.
- Universelle Connection-Manager-GUI: Profile anlegen, bearbeiten, duplizieren, löschen,
  importieren/exportieren, verbinden/trennen, testen und aktivieren. Domain-/Portkonflikte,
  Status, Controller, Mapping und Fähigkeiten sichtbar machen.
- Isaac Sim 5.1 als erste echte externe Simulation: lokale Installation/Launcher erkennen,
  separat starten, Original-SO-101 offiziell nach USD importieren, Szene/ROS-Graph erzeugen,
  reale JointStates zurücklesen und die vorhandenen Tasks mit Greifer ausführen.
  Physische Objektaufnahme gesondert prüfen, nicht aus einer Gelenkbewegung ableiten.
- Real Robot **ausschließlich read only**, auch wenn YAML manipuliert wurde. Domain-ID
  ist keine Bewegungsfreigabe. Generic ROS zunächst read only; Steuerung nur explizit
  als kompatible Simulation. MuJoCo/Gazebo als vorbereitete, ehrlich gekennzeichnete Plugins.
- Lokale Installations-/Launcher-/Szenenpfade nur in lokalen Settings bzw. Cache, portable
  Projektdateien enthalten logische Referenzen, Modellrevision, Mapping und ROS-Konfiguration.
- Optionaler Wizard-Eintrag für Integrationen; keine ungefragten großen Simulator-Downloads.

Vor diesem Auftrag vollständig verifiziert:
`a59c031` (Qt-Stabilisierung), `b9f381b` (Berichte), `cf71a51` (alte ACL-Bereinigung).
Git war zu Beginn dieses Auftrags sauber. Damals 137 Tests bestanden, 0 übersprungen,
52 Fensterzyklen inklusive 20 nativer SO-101/OpenGL-Zyklen, echte Humble-/Jazzy-/Lyrical-
Actions und vollständiger 13-Schritt-Task. Siehe `docs/GUI_STABILITY_REPORT.md`,
`docs/ROS_RUNTIME_VERIFICATION.md` und `docs/regression-verification.json`.
**Diese Vollabnahme gilt für den Ausgangspunkt, nicht automatisch für die jetzigen WIP-Änderungen.**

## 2. Tatsächlicher neuer Implementierungsstand

### Vorhandener Code, kurz geprüft, aber noch nicht vollständig integriert

- `backends/profiles.py`: Pydantic-`BackendProfile`, `new_profile`, portable Referenzen,
  Backend-ID unabhängig von Domain, Distribution, Netzprofil, Controller, Topics, Mapping,
  Modellbindung, Read-only-/Simulationsflags. Manipulierte Hardwareausführung wird abgewiesen.
  Runtime-/ExecutionConfig werden über bestehende Verträge erzeugt.
- `data/backends/defaults.json`: **einzige zentrale Vorschlagszuordnung**:
  Real Robot 0, Isaac 1, MuJoCo 2, Gazebo 3, ROS Demo 10, Generic 0.
  Werte werden nicht zur Backendidentität oder Sicherheitsentscheidung verwendet.
- `backends/domains.py`: `ROSDomainManager`, `DomainDiagnostic`, `validate_domain`.
  Fast-DDS-Standard-RTPS-Portformel (Basis 7400, Domainabstand 250, UDP-Maximum 65535),
  daraus maximal Domain 232; Warnung bei Überschneidung mit dem tatsächlich gelesenen
  Linux-Ephemeralbereich. Gemeinsame Domains benötigen explizite Bestätigung.
  Keine echte Mehrdomain-ROS-Verifikation bisher.
- `core/models.py`: optionale `Project.backends` und `active_backend`; eindeutige IDs und
  gültige Referenz werden geprüft. Format bleibt derzeit v3; alte v1/v2→v3-Migration besteht.
- `core/robot_state.py`: **bestehender** `RobotStateManager` um `backend_sources` erweitert.
  Neue `RobotState`-Metadaten `backend_id`, `ros_domain`, `ros_distribution`; `StateLease`
  kennt optional Backend-ID. `register_backend`, `backend_snapshot`, `backend_stale`,
  `activate_backend`, `remove_backend`, `_lease_state`. Inaktive Updates bleiben im
  jeweiligen internen Kontext; nur die aktive Projektion wird veröffentlicht. Gemeinsame
  URDF-/Mimic-/Zeit-/Zahlenvalidierung bleibt erhalten. Modellbindung leert Backendkontexte.
- `backends/plugins.py`: abstrakter `ROSBackendPlugin`, `BuiltinROSBackendPlugin`,
  `BackendPluginRegistry`, `BackendCapabilities`. Verwendet den bestehenden deklarativen
  PluginRegistry; keine dynamischen Fremdcode-Imports. Fähigkeiten werden anhand Typen,
  State, Controller, Clock und expliziter Simulationsfreigabe abgeleitet. Frische und
  Provenienz muss später der Manager zusätzlich liefern/prüfen.
- `plugins/registry.py`: neuer Manifest-Kind `backend`. Sechs YAML-Manifeste unter
  `data/plugins/backend-*.yaml`: ROS Demo, Isaac, Real Robot, Generic ROS, MuJoCo, Gazebo.
  MuJoCo/Gazebo sind `planned`. `implemented` bei anderen bedeutet vorhandenen Plugincode,
  **keine bestandene neue Backend-End-to-End-Abnahme**.
- `backends/transport.py`: `ReadOnlyRosbridgeTransport` blockiert Publishes/Goals und
  nicht freigegebene Serviceaufrufe an der ausgehenden Protokollgrenze. Erlaubt nur
  Subscribe/Unsubscribe und eine feste Rosapi-Discovery-Whitelist. Noch nicht GUI-verdrahtet.
- `runtime/manager.py`: neuer Payload-Modus `read_only`, bestätigtes Hostnetz auch für
  Read-only-Verbindungen; Backend-/Domainlabels. Standard-private Netzisolation bleibt.
- `data/runtime/entrypoint.sh`: Read-only startet nur rosbridge, keine Action-Gateways,
  Controller oder Demo-Publisher. Externe Simulation startet keinen alten Demo-Publisher.
- `data/runtime/controller_nodes.py`: Gateway-Topics verwenden optional
  `control_namespace`; Default `/studio/execution` hält die alte Strecke kompatibel.
- `motion/ros_backend.py`: optional `backend_id`-/Aktivitätsgate, Read-only-Sperre,
  konfigurierbarer Control-Namespace für Events, Commands und Heartbeat.
- `ros_bridge/websocket.py`: priorisiert auch neue `/studio/.../execution/...`-Controltopics.

### Isaac-Prototypen, noch nicht in Anwendung eingebunden

`backends/isaac.py` enthält:

- `IsaacSimInstallationManager`: Erkennen/Validieren mehrerer vorhandener 5.1-Installationen,
  offizieller Shell-/Pythonlauncher, Versionsdatei, Humble-/Jazzy-Bridge und URDF-Importer.
- `IsaacSimProcessManager`: eigene `QProcess`-Instanzen pro Backend, Domain/Middleware vor
  Start, getrennte Umgebungen, begrenzte Logtails, Status/Fehler. Stop kann ausschließlich
  selbst gespeicherte Prozesse terminieren. Robuste Shutdown-/Ready-/Restart-Integration
  und vollständige Prozess-/GUI-Tests fehlen noch.
- `IsaacSimSceneManager.prepare`: lokales Request-JSON mit unveränderter Original-URDF,
  Mapping, Modell-Signatur, Anfangswerten und FK-Vergleichsdaten; `validate_result` für
  spätere erzeugte Validierungsdaten. Keine absoluten Installationspfade im Projekt.

`data/isaac/studio_scene.py` ist ein **unverifizierter Standalone-Szenenprototyp** für Isaacs
`python.sh`. Keine Isaac-Imports im Desktop. Er sieht vor: `SimulationApp`, World,
Ground/Table/Light, offizieller URDF-Importer, USD-Referenz, SingleArticulation, Namen/Limits,
ROS2Context mit Request-Domain, JointState-Publisher/-Subscriber, ArticulationController,
Clock, Kamera, Szenenspeicherung und kontrollierte Prozessschleife. Dies wurde **noch nicht
ausgeführt**. APIs, Importer-Pfade, Schema, Physik, Nullpose und Feedback müssen geprüft werden.
Die Simulations-Drivewerte 1000/50 sind Prototyp-Tuningwerte, keine Hardwarekalibrierung.

### Noch nicht begonnen/fertig

- **Es gibt noch keine Klasse `ROSBackendManager`, kein `BackendSession` und keine
  `backends/manager.py`.** Dynamische Portverwaltung und Container-pro-Profil-Orchestrierung fehlen.
- Kein Connection-Manager-Dialog, keine Backend-Auswahl, keine neue MainWindow-Integration,
  keine Wizard-Erweiterung. Die bestehende Single-Session-ROS-GUI ist unverändert.
- Keine Persistenz der lokalen Installations-/Szenenzuordnung in Settings verdrahtet.
- Keine echten gleichzeitigen Backend-Verbindungen oder Domain-Isolationstests.
- Keine erzeugte/validierte SO-101-USD-Szene, kein neuer Isaac-Joint-/Task-Bewegungstest.
- Neue Module besitzen noch keine umfassenden eigenen automatisierten Regressionstests.

## 3. Letzter Arbeitsschritt und wichtige Anschlussentscheidungen

Zuletzt bearbeitet: `data/isaac/studio_scene.py` und `backends/isaac.py`.
Es wurde der Standalone-Importer-/Szenenentwurf geschrieben. Bei der Pausensicherung wurde
nur noch der nachweislich falsche Aufruf `set_joint_gains` auf die tatsächlich installierte
API `ArticulationController.set_gains` korrigiert. Dieser Szenenpfad bleibt ungetestet.

**Wichtige Stolperstelle:** Ein erster Request liegt in
`.bootstrap/isaac-so101/scene-request.json`. Dieser wurde mit `pick_place_demo()` **ohne**
RobotModelManager erstellt und enthält tatsächlich den **generischen** Demoroboter
(`project.robot.model_id is None`). Der Verzeichnisname ist irreführend. Nicht als
SO-101-Nachweis benutzen. Kein USD wurde dadurch generiert; Datei wurde nicht gelöscht.
Für die echte Fortsetzung `pick_place_demo(RobotModelManager())` verwenden und explizit
`robot.model_id == 'so101_follower'` sowie Quellrevision prüfen.

Bereits überlegte, **noch nicht implementierte** Anschlussarchitektur:

1. Ein `ROSBackendManager(QObject)` orchestriert pro Profil eine Session mit eigenem
   RosbridgeTransport, Container-ID, SourceLease, Graph, Controllerstatus, Clock und Generation.
   Bestehenden RuntimeManager/TaskRunner wiederverwenden; blockierende Docker-I/O in Worker.
2. Serialisierte Start-/Stop-/Neustartoperationen, kontrollierter Abbruch auch bei Connect-
   Races; verspätete Startresultate müssen eigene Container wieder bereinigen. Keine Adoption
   oder Beendigung fremder Prozesse/Container.
3. Dynamische bzw. kontrollierte Ports mit eindeutiger Zuordnung und Konfliktdiagnose.
   Jeder Backend-Control-Namespace muss aus einer eindeutigen, ROS-namensgültigen ID
   abgeleitet werden. Die jetzige Payload-Unterstützung allein erzeugt noch keinen solchen.
4. Bestehende GUI-Transportverdrahtung (`MainWindow.attach_transport/detach_transport`)
   beendet und löscht momentan immer den alten Transport und trennt alle Signale.
   Für Hintergrundverbindungen muss nur die aktive GUI-Projektion abgekoppelt werden;
   Manager-Abonnements und andere Sessions dürfen weiterlaufen. Alten Code nicht blind übernehmen.
5. Vor Aktivierung alte Ausführung abbrechen, dann neuen Lease/State projizieren; Befehle
   nur aktivem, frischem, modell-/mappingvalidiertem Simulationsbackend erlauben.
   Real Robot bleibt mehrfach abgesichert read only, unabhängig von Domain und YAML.
6. `RobotStatePanel.receive` verwendet bislang den Puffer-Quellschlüssel `(instance,backend)`.
   Vor Multi-Backend-GUI unbedingt `backend_id`/Domain einbeziehen, damit zwei gleiche
   Backendtypen keine Plotdaten mischen. Keine neue parallele State-Managerklasse einführen.
7. Hostnetz ist nur explizit bestätigten externen Profilen erlaubt; ROS Demo bleibt im
   privaten Docker-Netz. Ein Domain-Isolationstest auf getrennten privaten Netzen allein
   beweist keine DDS-Domain-Isolation: kontrollierte Testnodes müssen auch ein geeignetes
   gemeinsames Testnetz benutzen, ohne Hardware zu berühren.
8. MainWindow-Close muss alle selbst gestarteten Backendverbindungen/Worker/Isaac-Prozesse
   kontrolliert herunterfahren; die bereits reparierte Qt-/Plot-/Shader-Lebensdauer erhalten.
9. `BackendCapabilities` ersetzt keine ausführliche Endpunkt-/Frische-/Modellprüfung.
   Erkennung eines Topics oder einer Domain darf keine Hardwarefreigabe ergeben.

## 4. Isaac-Installation und tatsächlich erfolgte Probe

- Installation: `$HOME/isaacsim`.
- VERSION: `5.1.0-rc.19+release.26219.9c81211b.gl`.
- Offizielle Launcher vorhanden und ausführbar: `isaac-sim.sh`, `python.sh`.
- Lokale Erweiterungen vorhanden: `exts/isaacsim.asset.importer.urdf`,
  `exts/isaacsim.ros2.bridge` mit Humble/Jazzy-Unterverzeichnissen.
- GPU: NVIDIA GeForce RTX 4070 Ti, 12 GiB; Treiber 580.178.04.
- Erfolgreich ausgeführt: separater headless `SimulationApp`-Start über `python.sh`,
  Enable der Importer- und ROS-Bridge-Erweiterungen, zehn App-Updates, kontrolliertes Close.
  Marker **`STUDIO_ISAAC_PROBE_READY`**, Exit 0. Bridge **4.12.4** lud internes Humble-rclpy;
  keine System-rclpy-/Desktop-venv-Vermischung. Domain vor Start **1**, Fast DDS.
- Probe-Skript `/tmp/studio_isaac_probe.py`, Log `/tmp/studio-isaac-probe.log` sind temporär
  und nicht in Git. Die obigen Ergebnisse sind hier dauerhaft dokumentiert.
- Diese Probe importierte **keinen Roboter**, erzeugte **kein USD** und führte **keine
  Gelenkbewegung oder ROS-End-to-End-Verbindung** aus.
- Original-SO-101 ist im Cache installiert:
  `$HOME/.local/share/ros-robot-studio/robots/so101_follower/a758567c3978dfeefe282ede0500085a48fe8f78/so101_new_calib.urdf`.
  Original-URDF/Meshes wurden nicht verändert.

Lokale API-Quellen für die Fortsetzung:
`standalone_examples/api/isaacsim.asset.importer.urdf/urdf_import.py`,
`standalone_examples/api/isaacsim.ros2.bridge/moveit.py`, `clock.py`,
`exts/isaacsim.ros2.bridge/ogn/docs/OgnROS2Context.rst`.
Insbesondere Contextinputs `domain_id` und `useDomainIDEnvVar`.

Offizielle versionsgebundene Referenzen:
[Importer](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/importer_exporter/ext_isaacsim_asset_importer_urdf.html),
[Importer API](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.asset.importer.urdf/docs/index.html),
[ROS Joint Control](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_manipulation.html),
[ROS Installation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html).
Für DDS-Portregeln ROS-2-Primärdokumentation oder deren Quellrepository verwenden;
docs.ros.org antwortete in dieser Sitzung beim Domain-Dokument mit Zugriffsschutz.

## 5. Docker-/ROS- und Prozesszustand bei der Pause

Docker Engine **29.9.3**, Zugriff nach Benutzeraktion wieder vorhanden.
Installierte, **nicht gelöschte und in diesem Auftrag noch nicht neu gebaute** Images:

| Image | Image-ID (kurz) |
| --- | --- |
| ros-robot-studio/humble:0.1.0 | d14bbc582a83 |
| ros-robot-studio/jazzy:0.1.0 | 73ff3e8b3ff1 |
| ros-robot-studio/lyrical:0.1.0 | 2250265f366b |

**Die Images sind für die geänderten Runtime-Quellen jetzt veraltet.**
Alter eingebauter Sourcehash `b230f63be058ca8f0a4bf4fbe63bf438881f33dbbe5da517cbe28289c6f868a7`;
jetziger Sourcehash `a4e69f910a3413a104cfb1bd8708d49d808d2151ee4274945cdc8e18c33c0ef3`.
`RuntimeManager.status().installed` kann daher trotz vorhandener Images leer sein.
Vor echten neuen ROS-Tests regulär neu bauen/installieren; keinen Hashcheck abschalten
und alte Verified-Marker nicht als Beweis für den neuen Build verwenden.

Domain 1 wurde nur für den Isaac-Startup verwendet. Domains 1/4 wurden zusätzlich rein
im Speicher für State-/Profilchecks verwendet, keine echte DDS-Nachrichtenstrecke.
Domains 0/2/3/10 sind neue Vorschläge, nicht neu als ROS-Verbindung abgenommen.
Bestehende Demo verwendet bislang standardmäßig Domain 42/private Docker-Netze.
Externe Profile sollen Hostnetz/Fast-DDS-UDP nach expliziter Konfiguration verwenden.

Bei der abschließenden Prozesskontrolle:

- **0 Studio-Container, 0 Studio-Testnetze**.
- Keine laufenden zugehörigen `studio_isaac_probe`, `studio_scene`, Verifikations- oder
  Pytest-Prozesse gefunden. Isaac-Probe war bereits regulär beendet; nichts abzuschießen.
- Keine fremden Container/Simulatorprozesse verändert, keine Images oder USD-Dateien gelöscht.
- Temporäre Socket-ACL **`user:local-user:rw-` ist weiterhin vorhanden**. Der Benutzer hatte sie
  für diesen Auftrag erneut gesetzt und die Freigabe bestätigt. Sie wurde zur Pause
  ausdrücklich **nicht** entfernt oder sonst verändert. Nach Fortsetzung erneut prüfen
  und nach endgültigem Testabschluss regulär mit Benutzerhilfe bereinigen.
- Agentprozess besitzt weiterhin alte Gruppen und `NoNewPrivs`; keine sudo-/Gruppen-
  oder Socketberechtigungsumgehung. Dauerhafte Gruppenaufnahme war bereits früher erfolgt;
  für passende neue Desktopprozesse ist eine neue Anmeldung nötig.

## 6. Kurze Prüfungen am Pausenstand

Tatsächlich abgeschlossen:

```bash
env -u PYTHONPATH QT_QPA_PLATFORM=offscreen .venv/bin/pytest -q \
  tests/test_robot_state.py tests/test_ros_execution.py tests/test_runtime.py \
  tests/test_projects.py tests/test_robot_plugins.py
```

**51 bestanden in 1,87 s, 0 fehlgeschlagen, 0 übersprungen.**
Davor separat 7 Robot-State- und 19 ROS-Execution-/Runtime-Tests bestanden (Teilmengen,
nicht zu einer größeren Testanzahl addieren).

- Ruff: bestanden; Formatprüfung: **98 Dateien** bestanden.
- `python -m compileall -q src scripts`: bestanden.
- `bash -n src/ros_robot_studio/data/runtime/entrypoint.sh`: bestanden.
- Kurzer Inline-Vertragstest: zwei Isaac-Profile mit unterschiedlichen IDs/Domains,
  Project-Roundtrip, Hardwareausführungs-Ablehnung, Domain 233 abgewiesen, getrennte
  Hintergrundstates, aktiver Statewechsel und Ablehnung entfernter Quell-Leases: bestanden.
  Das ist kein Docker-/DDS-Test und noch keine neue persistierte Pytest-Suite.
- Offscreen-Anwendungsstart mit `--smoke-test`, isolierten temporären XDG-Pfaden: **Exit 0**.
  Keine neue native Multi-Backend-GUI-Abnahme, weil diese GUI noch nicht existiert.
- `git diff --check`: bestanden; auch der gestagte Diff wird vor dem WIP-Commit geprüft.

Gelöste Prüf-/Aufrufprobleme, nicht als bestandene Tests zählen:

- Ein erster Pytest-Aufruf nannte irrtümlich die nicht existierende `tests/test_core.py`;
  keine Tests liefen. Danach korrekte Dateien und obiger 51-Test-Lauf.
- Ein erster Offscreen-Smoketest scheiterte an Sandbox-Schreibschutz für das normale
  XDG-Logverzeichnis. Wiederholung mit `/tmp/studio-pause-{state,config,data}` bestand.
- Isaacs API-Imports benötigen eine bereits initialisierte SimulationApp. Der Prototyp
  wurde in `main()` gekapselt; dadurch kein E402-Lintfehler und keine SDK-Imports beim
  bloßen Importieren des Desktopmoduls. Keine Runtime-Warnungen/Crashes unterdrückt.

Nicht ausgeführt: voller 137+-Regressionslauf, neue Docker-Builds, neue Domain-/Port-/
Parallelverbindungs-/Hardware-Readonly-ROS-Tests, SO-101-USD-Import, PhysX-/Neutralpose-/
Achsenvergleich, Isaac-Jointsteuerung, Pick-and-Place, neue GUI-/Qt/OpenGL-Lebensdauerabnahme.
Keine dieser Funktionen als verifiziert ausgeben.

Paketstand unverändert: Python 3.10, PySide6/Addons/Essentials/Shiboken **6.9.3**,
PyQtGraph **0.13.7**, NumPy **2.2.6**, PyOpenGL **3.1.9**, SciPy **1.15.3**.
Keine globalen Pythonpakete installiert oder aktualisiert.

## 7. Konkrete nächste Schritte nach ausdrücklichem Fortsetzungsprompt

1. Diesen WIP und den vollständigen Entwicklungsauftrag lesen, `git status`/`git log`
   prüfen; Änderungen erhalten. Dockerzugriff und ACL lesen, keine Berechtigungen eigenmächtig ändern.
2. Eigene gezielte Tests für Profile/Domainkonflikte/Portregeln/Read-only-Protokollgrenze/
   Backend-State-Leases ergänzen. Registrierungs-, Modellwechsel-, Zeitstempel-, Lösch- und
   Aktivierungsraces prüfen. Profilvalidierung von Bool/Int und portable Import-/Exportgrenzen beachten.
3. `backends/manager.py` mit `ROSBackendManager` und pro Profil gekapselten Sessions bauen:
   serialisierte Worker, Portverwaltung, getrennte Transports/Graphs, eindeutige Controltopics,
   nur eine aktive Commandroute, Modell-/Mapping-/Controller-/Frischevalidierung, sichere Bereinigung.
4. MainWindow/Connection-Manager-GUI und lokale Settings anbinden. Bestehende Demo-/ROS-
   Umschaltung weiterverwenden, Hintergrundtransports beim Wechsel behalten, Quellenidentität
   im Monitoring erweitern. Projektwechsel und Close koordinieren, Hardware sperren.
5. Geänderte Runtime-Images regulär bauen und echte Domain-Isolation mit kontrollierten
   Testnodes nachweisen. Read-only-Hardware ausschließlich mit Mocks/Testsystem prüfen.
6. Echte SO-101-Requestdaten aus RobotModelManager erstellen; Szenenprototyp über offiziellen
   Isaac-Launcher ausführen und dessen Fehler einzeln beheben. Achsen/Limits/Skalierung,
   Nullpose/FK, Articulation, Greifer und Scene-Cache/Referenzen validieren. Originaldateien erhalten.
7. Isaac Domain 1 + ROS-Demo Domain 10 gleichzeitig verbinden, kleine beobachtete Bewegung,
   dann denselben vorhandenen Task mit Greifer; Commandrouting, Abbruch, Clockpause/-reset,
   Zielabweichung und Backendwechsel wirklich prüfen. Keine erfundene physische Objektaufnahme.
8. Optionalen Wizard, portable Profile/Mapping-Zuordnung, generisches Read-only-Backend und
   ehrlich vorbereitete MuJoCo-/Gazebo-Schnittstellen vervollständigen.
9. Erst anschließend vollständige Regression, native Qt/OpenGL-/Mehrfensterprüfung,
   Humble/Jazzy/Lyrical, Screenshots, geforderte Backend-Dokumentation und lokale getrennte
   Commits. Keinen Push. Temporäre Testressourcen/ACL am wirklichen Abschluss bereinigen.

## 8. Sicherer Zustand für das Update

Alle Dateioperationen abgeschlossen; Prototypen als WIP bewusst erhalten. Quellen,
Konfigurationen und dieses Dokument liegen im Repository. `.bootstrap`-Requests und
vorhandene Modell-/Simulatorinstallationen bleiben erhalten; keine großen Assets/Logs
werden committed. Die neue Architektur ist **noch nicht fertig nutzbar**, aber die kurzen
Prüfungen und der bestehende Offline-Start funktionieren. Runtime-Images benötigen vor
weiteren ROS-Läufen einen Neubau.

**Die Entwicklung kann an diesem dokumentierten Punkt sicher unterbrochen werden.**
Keine automatische Fortsetzung nach dieser Übergabe.
