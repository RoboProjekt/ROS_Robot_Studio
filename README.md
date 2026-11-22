# ROS Robot Studio

ROS Robot Studio ist eine modulare Desktop-Anwendung für Roboterkonfiguration, Kinematik, Aufgabenplanung und ROS-2-Simulation. Ein Projekt verbindet URDF, Gelenkstellungen, gespeicherte Positionen, Tasks, ROS-Backends und Kameraprofile. Der SO-101 Follower ist das Referenzmodell für die Offline- und Isaac-Sim-Workflows.

![ROS Robot Studio mit SO-101 im 3D-Viewer und Live Monitoring](docs/images/overview/robot-studio-main.png)

*Der SO-101 Follower in Robot Configuration neben dem Live Monitoring. Beide Ansichten verwenden denselben aktiven Roboterzustand.*

- [ROS Robot Studio – Overview & Features](#ros-robot-studio--overview--features)
- [Projekte und Isaac-Szenen öffnen](#projekte-und-isaac-szenen-öffnen)
- [Tutorial – SO-101 in ROS Robot Studio und Isaac Sim](#tutorial--so-101-in-ros-robot-studio-und-isaac-sim)
  - [Installieren und den SO-101 konfigurieren](#1-installation)
  - [Humble, Isaac Sim und eine eigene USD](#6-ros-2-humble-einrichten)
  - [Gelenkbewegung, Positionen und Tasks](#11-gelenkbewegung-und-live-monitoring)
  - [RGB-Kamera und Objekterkennung](#14-rgb-kamera-zur-usd-hinzufügen)
  - [Eigene Roboter und Szenen](#eigene-roboter-und-szenen)
- [Physischen SO-101 Follower über USB anbinden](#physischen-so-101-follower-über-usb-anbinden)
- [Lokale MuJoCo-Runtime](#lokale-mujoco-runtime)

## ROS Robot Studio – Overview & Features

| Bereich | Status | Funktionsumfang |
| --- | --- | --- |
| Robot Configuration | Supported | URDF-Import, native OpenGL-Meshes, Link- und Gelenkbaum, URDF-Limits, virtuelle Gelenkregler und aktuelle Werkzeugpose |
| Kinematics & Motion | Supported | Universelle FK, validierte Ketten, SciPy-IK mit Gelenkgrenzen, Zielvorschau und Poseanwendung offline |
| Positions & Tasks | Supported | Teach-in, Joint- und Cartesian-Positionen, Taskeditor, relative Endpunkte, Greiferaktionen und Pick-and-Place-Wizard |
| ROS Runtime | Supported | Verwaltete Humble-, Jazzy- und Lyrical-Container, rosbridge/rosapi, DDS-Domains und Wechsel des aktiven Backends |
| Simulation | Supported | Isaac Sim 5.1 mit Humble, USD-Aufbau aus URDF, native JointStates, Zeitbasis und Trajektorienausführung über den Isaac-Adapter |
| Live Monitoring | Supported | Positionen, Geschwindigkeiten, Roboterzustand, ROS-Graph, Taskstatus und Zeitreihen |
| Dockable Workspace | Supported | Haupttabs ziehen, horizontal oder vertikal teilen, Layout speichern und wiederherstellen |
| Camera & Vision | Supported | ROS-RGB-Kameras, feste Isaac-Kamera, HSV, ArUco, YOLOv5n v7 ONNX, CameraInfo/TF2 und planare Positionen |
| Vision PoseProvider | Supported | Gültigen Punkt in einen manuellen Zielentwurf mit expliziter Werkzeugorientierung und Versatz übernehmen |
| Real Robot | USB manual / ROS read only | ROS-Gelenkdaten überwachen, SO-101-Positionen über USB lesen und mit dem mitgelieferten Projekt einzelne Gelenke bewegen |
| Physischer Objektkontakt, bewegte Kamera-Montagen, Homographie | Planned | Greif- und Kontaktvalidierung sowie zusätzliche Kalibrierungs- und Montageverfahren |
| MuJoCo | Supported | Lokale Simulation aus URDF oder MJCF mit Gelenkrückmeldung und Positionsreglern |
| Gazebo | Planned | Profilverwaltung und lokale Dateizuordnung sind vorbereitet. |

### Funktionsansichten

| Robot Configuration | Kinematics & Motion |
| --- | --- |
| ![SO-101 mit originalen Meshes und Gelenkreglern](docs/images/features/robot-configuration.png) | ![SO-101-Kette, Endeffektor und IK-Solver](docs/images/features/kinematics.png) |
| *Originalgeometrie, aktuelle Gelenkstellung und Modellstruktur.* | *Kettenkonfiguration für den fünf Freiheitsgrade umfassenden Arm.* |

| Positions & Tasks | ROS Runtime und Simulation Runtime |
| --- | --- |
| ![Taskeditor mit Gelenkbewegung, Wartezeit und Rückkehr](docs/images/features/task-editor.png) | ![Humble und NVIDIA Isaac Sim im Runtime-Banner](docs/images/features/simulation-runtime.png) |
| *Taskschritte und ihre Validierung in einem Projekt.* | *Runtime, Szenenauswahl und Live-Status im Banner.* |

| Live Monitoring | Dockable Workspace |
| --- | --- |
| ![JointState-Rückmeldung und Monitoring während einer SO-101-Bewegung](docs/images/features/live-monitoring.png) | ![Horizontal geteilter Studio-Workspace](docs/images/features/dockable-workspace.png) |
| *Live-Zustand und Bewegungsfortschritt aus Isaac Sim.* | *Zwei Hauptansichten in einer gespeicherten Fensteranordnung.* |

![RGB-Livebild mit HSV-Overlay und planarem Zielpunkt](docs/images/features/camera-vision.png)

*Camera & Vision zeigt das Isaac-RGB-Bild, den HSV-Fund und seine Position auf der bekannten Tischfläche.*

### Getestete Umgebung

| Software | Getesteter Stand und Einsatz |
| --- | --- |
| Ubuntu | 22.04 LTS, x86-64, X11 für native OpenGL-Abnahme |
| Python | 3.10 für die Desktop-Anwendung |
| PySide6 / Qt | 6.9.3 aus dem lokalen Python-Environment |
| Docker Engine | 29.9.3 bei der Systemabnahme. Benötigt für die verwalteten ROS-Laufzeiten. |
| ROS 2 Humble | Eigener Container, ROS-Demo und getestete Isaac-Sim-/Vision-Kombination |
| ROS 2 Jazzy | Eigener Container und ROS-Demo geprüft. Der Isaac-Manager prüft die lokal vorhandene Jazzy-Bridge. |
| ROS 2 Lyrical | Eigener Container und ROS-Demo geprüft. Isaac-Adapter unterstützt Humble/Jazzy. |
| NVIDIA Isaac Sim | 5.1, optionaler eigener Prozess. Abnahme mit Humble und NVIDIA-GPU. |
| MuJoCo | 3.8.1 im Studio-Python, optionales Paket |
| Gazebo | Ausführung geplant |

Für eigene URDFs sind Import, FK, Kettenanalyse und der modellabhängige IK-Workflow verfügbar. Modellgrenzen, Controller, Gelenkmapping und erreichbare Ziele werden für den jeweiligen Roboter geprüft.

### Architektur

```mermaid
flowchart LR
    UI["Desktop mit Qt und OpenGL"] --> Model["URDF und Kinematik"]
    UI --> Tasks["Positionen und Taskeditor"]
    Tasks --> Offline["Offline-Simulation"]
    Tasks --> Runtime["ROS Backend Manager"]
    UI --> Runtime
    Runtime --> ROS["ROS 2 und rosbridge im Docker-Container"]
    UI --> Isaac["Eigener Isaac-Sim-Prozess"]
    ROS <-->|"JointStates, Kommandos und Clock"| Isaac
    Model --> State["Gemeinsamer Roboterzustand"]
    Offline --> State
    ROS --> State
    State --> Monitor["Live Monitoring und 3D-Viewer"]
    ROS --> RGB["RGB-Worker mit CameraInfo und TF2"]
    RGB --> Vision["Detektoren und planare Lokalisierung"]
```

[Architektur](ARCHITECTURE.md) · [Backend-Profile](docs/BACKEND_CONNECTION_MANAGER.md) · [Kinematik](docs/KINEMATICS.md) · [Vision-Abnahme](docs/VISION_VERIFICATION.md)

## Tutorial – SO-101 in ROS Robot Studio und Isaac Sim

Die Schritte 1–5 funktionieren offline. Ab Schritt 6 verwendet das Beispiel einen verwalteten Humble-Container. Für Isaac Sim wird der originale SO-101 Follower aus dem Studio-Katalog geladen.

### 1. Installation

1. Auf Ubuntu 22.04 Git, Python 3.10, das passende `venv`-Modul und eine native Qt/OpenGL-Umgebung bereitstellen. Das Repository klonen und im Projektordner ein Terminal öffnen.
2. `./start.sh` im Projektordner ausführen. Der Launcher erstellt `.venv`, installiert die gesperrten Python-Abhängigkeiten und öffnet den Setup-Assistenten.
3. Für den Offline-Einstieg **Open offline demo** wählen. Über **Help → Setup assistant** lässt sich der Assistent später erneut öffnen.
4. Für den Simulationsabschnitt Docker Engine installieren und starten. Bei einem vorhandenen systemweiten Dienst zeigt `systemctl is-active docker` den Zustand. `docker info` muss aus derselben Benutzersitzung wie Studio funktionieren. Die Mitgliedschaft in der Docker-Gruppe gewährt weitreichende Hostrechte und wird erst nach einer neuen Anmeldung wirksam. Die Anwendung als normaler Benutzer starten.
5. In **Setup assistant → Components** über **Install Universal Kinematics Support** den SciPy-Solver installieren. **Install optional RGB detector support** ergänzt OpenCV und ONNX Runtime für Schritt 16.

```bash
git clone https://github.com/RoboProjekt/ROS_Robot_Studio.git
cd ROS_Robot_Studio
./start.sh
```

![Setup-Assistent von ROS Robot Studio](docs/images/tutorial/00-setup-assistant.png)

*Der Setup-Assistent prüft die Umgebung. Offline-Arbeit kann direkt gestartet werden.*

[Installationsanleitung](INSTALLATION.md) · [Docker-Diagnose](docs/DOCKER_DIAGNOSTICS.md) · [Isaac Sim 5.1 herunterladen](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html) · [NVIDIA-Systemanforderungen](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html)

### 2. Projekt anlegen

1. Oben **New project** klicken. Studio öffnet den Arbeitsbereich **Robot Configuration**.
2. Im Reiter **Robot model** unter **Project name** den Namen `SO-101 Lab` eingeben.
3. **Save** klicken und beispielsweise `SO101-Lab.studio.yaml` in einem eigenen Projektordner speichern.

![Neues Studio-Projekt mit dem Namen SO-101 Lab](docs/images/tutorial/01-create-project.png)

*Der Projektname wird im Modellreiter eingegeben. Save öffnet beim ersten Speichern den Dateidialog.*

### 3. Originalen SO-101 laden oder eine URDF importieren

1. **Robot Configuration → Robot model → Select robot → SO-101 Follower** wählen.
2. Falls das Modell fehlt, **Install Robot Model** anklicken und den Download bestätigen. Studio holt die fixierte Follower-Revision einschließlich der originalen STL-Dateien und prüft die Prüfsummen.
3. **Meshes** aktivieren. Der Viewer sollte **17/17 visuals** anzeigen. Unter **Robot structure** stehen acht Links und sieben Joints, darunter das feste Gelenk zum Werkzeugframe.
4. Für eine manuelle URDF-Variante **Import URDF / Xacro** klicken und `Simulation/SO101/so101_new_calib.urdf` mit dem zugehörigen `assets/`-Ordner wählen. Der Katalogweg übernimmt zusätzlich die Modellrevision und die mitgelieferte Greiferkonfiguration. Für die folgenden Schritte wieder **SO-101 Follower** auswählen.

![Geladener SO-101 mit Gelenkstruktur und originaler Geometrie](docs/images/tutorial/04-so101-loaded.png)

*Der Modellreiter zeigt die Quellenrevision, die visualisierten Geometrien und den Gelenkbaum.*

![SO-101-URDF im Qt-Dateidialog](docs/images/tutorial/02-urdf-picker.png)

*Die Auswahl zeigt die Datei so101_new_calib.urdf.*

![Manuell importierte SO-101-URDF im Robot model](docs/images/tutorial/03-urdf-imported.png)

*Der direkte Import zeigt die Geometrie als benutzerdefiniertes URDF-Modell.*

[Fixierte Originalquelle und Lizenz](docs/SO101.md)

### 4. Gelenke und Limits prüfen

Im Offline-Modus unter **Virtual joints** den Regler für `shoulder_pan` bewegen oder im Zahlenfeld `5.73°` eingeben. Die Basis dreht sich und die Werkzeugpose ändert sich. Revolute-Werte werden in Grad angezeigt und intern in Radiant gespeichert. Die sechs beweglichen Achsen sind `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll` und `gripper`. Mit **Zero / default pose** zur URDF-Startkonfiguration zurückkehren.

![Virtuelle Gelenkregler mit veränderter SO-101-Pose](docs/images/tutorial/05-joint-controls.png)

*Eine Gelenkänderung aktualisiert die Original-Meshes und die angezeigte Endeffektorpose.*

### 5. Kinematik einrichten

1. **Robot Configuration → Kinematics & Motion → Chain setup** öffnen.
2. **Base frame** auf `base_link`, **End-effector frame** auf `gripper_frame_link` und **Gripper joint** auf `gripper` setzen.
3. **Offline solver → Automatic · SciPy bounded IK** auswählen und **Confirm chain & run FK / IK checks** klicken. Die Liste **Active arm joints** enthält fünf Armachsen.
4. Unter **Cartesian control** zunächst die aktuelle Position als Ziel verwenden, **Position Only** wählen und **Calculate IK** klicken. **Preview Pose** zeigt die Lösung im Viewer. **Apply to Offline Model** übernimmt sie in den virtuellen Zustand.
5. Für einen ersten Test die Zielposition nur wenige Millimeter verändern und das Resultat samt Restfehler prüfen. Eine frei wählbare volle 6D-Pose erfordert zusätzliche Freiheitsgrade. Der SO-101 bietet eine separate Greiferachse.

![Chain setup für base_link, gripper_frame_link und SciPy](docs/images/tutorial/06-chain-setup.png)

*Die bestätigte SO-101-Kette trennt den Greifer von den fünf Armachsen.*

[Solver-Auswahl im nativen Studio](docs/images/tutorial/07-solver-selection.png)

### 6. ROS 2 Humble einrichten

1. Im **Setup assistant** auf der Seite **ROS runtimes** Humble auswählen und **Install selected runtimes** ausführen. Vorhandene Images werden im Laufzeitstatus angezeigt.
2. Oben **ROS Runtime → ROS 2 Humble** und für den ersten Test **Start ROS demo** wählen. **Project & Runtime** zeigt Container und Verbindungszustand. **Live Monitoring** zeigt den aktiven Zustand. **Stop** beendet die Studio-Demo vor dem Wechsel zu Isaac.
3. Bei Zugriffsfehlern die eingeblendete Docker-Diagnose öffnen. Sie unterscheidet fehlenden Daemon, Socketzugriff und Gruppen im tatsächlich laufenden Studioprozess.

Die Distribution bestimmt ROS-Pakete und Containerbasis. **ROS_DOMAIN_ID** trennt DDS-Teilnehmer. Ein Backendprofil speichert Domain, Distribution, Topics und Modellbindung. Für dieses Tutorial verwendet Isaac **Domain 1**, die ROS-Demo ein separates Profil.

![Projekt- und Runtime-Ansicht mit aktivem Humble-Container](docs/images/tutorial/08-ros-runtime.png)

*Project & Runtime zeigt die installierte Distribution und den Zustand der Studio-Verbindung.*

![Docker-Diagnose mit erreichbarem Socket und Modus 0660](docs/docker-diagnostics.png)

*Der Ausschnitt zeigt den tatsächlich verwendeten Docker-Endpoint und Socketzugriff.*

### 7. Isaac-Sim-Profil konfigurieren

1. In der Toolbar **Backends… → Add** öffnen.
2. Im Reiter **Connection** `isaac_sim`, `local_installation`, `humble`, **ROS_DOMAIN_ID** `1`, **Network profile** `host` und **Discovery** `localhost` wählen. **Local Isaac installation** auf den entpackten Isaac-Sim-5.1-Ordner mit `isaac-sim.sh` und `python.sh` setzen. **Detect Isaac Sim 5.1** kann einen bekannten Installationspfad übernehmen.
3. Für die eigene Simulation **Allow host DDS networking for this connection**, **I confirm this endpoint is a simulation with no robot hardware** und **Request simulation execution** aktivieren. **Read only · no motion commands** für diese reine Simulationsverbindung abwählen.
4. In **Robot / interfaces** `joint_state_command` und **Use simulation time** prüfen. Das Mapping bleibt für den SO-101 identisch. **Bind this profile and mapping to the currently loaded robot** aktivieren und **Save** klicken. Für sichtbare Tests **Run owned Isaac process headless** abgewählt lassen.

![Backend Connection Manager mit dem SO-101-Isaac-Profil](docs/images/tutorial/09-backend-manager.png)

*Die Tabelle zeigt die Profilart, DDS-Domain und ROS-Distribution.*

![Isaac-Profil mit Humble, Domain 1 und lokalem Setup](docs/images/tutorial/10-isaac-profile.png)

*Connection enthält die lokalen Pfade. Robot / interfaces enthält das Modellmapping und die Topics.*

### 8. Eigene USD aus der URDF erzeugen und bearbeiten

1. Optional im Profil unter **Generated scene directory (optional)** einen Ordner für lokale Szenen angeben. **Existing USD scene (optional)** für den Neuaufbau leer lassen.
2. Im Runtime-Banner unter **Simulation Runtime** das Isaac-Profil und unter **Simulation File → Generate from current robot** wählen. Auf **Start Simulation** klicken, um den Isaac-Import zu starten. Das Programm erzeugt eine neue Stage in einem eigenen Ausgabeordner und importiert die aktuell gebundene URDF samt Meshes.
3. Im geöffneten Isaac-Fenster im **Stage**-Baum **World** aufklappen. Dort liegen der Roboter unter `/World/Robot`, der kollidierbare `MountingTable`, der Boden, das Licht und der ROS-Graph. Die neue Stage verwendet Meter und Z-up. Der feste Roboterfuß bleibt im Beispiel am Ursprung auf dem Tisch.
4. Den Robot-Unterbaum aufklappen und im **Property**-Bereich die Articulation und Joint-Drives prüfen. Der Import übernimmt Gelenknamen, Achsen, Limits und Antriebskraftgrenzen aus der URDF. Studio setzt Kraftantriebe mit effektiver Steifigkeit 5000 und Dämpfung 50 in SI-Einheiten. Rotations-Drives werden für USD-Gradwerte umgerechnet. Die Simulation verwendet normale Schwerkraft. Die erzeugte Szene enthält Physikszene, Robotercollider und Tischcollision.
5. Für eine eigene Anordnung beispielsweise den vorhandenen **Light**-Prim auswählen und seine Intensität im **Property**-Bereich von 800 auf 900 ändern. Umgebung und Beleuchtung können auch nach dem Import bearbeitet werden. Für dieses Beispiel Articulation, Roboterbasis, Joint-Drives und Tischhöhe beibehalten. In Isaac über **File → Save As…** beispielsweise `SO101-Lab.usda` im Ausgabeordner speichern oder die erzeugte `scene.usd` als Basis wählen. Den vollständigen generierten Pfad zeigt Studio unter **Project & Runtime** im Protokoll als `Generated Isaac USD`.

![Startauswahl für die Erzeugung einer USD aus dem aktuellen Robotermodell](docs/images/tutorial/11-generate-scene.png)

*Generate from current robot erzeugt eine neue Stage aus dem gebundenen SO-101.*

![Isaac Sim mit SO-101, Stage Tree, Montagetisch und ROS-Graph](docs/images/tutorial/12-isaac-stage.png)

*Der von Studio erzeugte Stage-Tree im tatsächlichen Isaac-Sim-Fenster.*

| Articulation | Schultergelenk |
| --- | --- |
| ![Ausgewählte Articulation im Isaac Stage Tree](docs/images/tutorial/13-isaac-articulation.png) | ![Ausgewähltes Schultergelenk und sein Prim-Pfad in Isaac Sim](docs/images/tutorial/14-isaac-joint.png) |
| *Die Auswahl zeigt den importierten Robot-Unterbaum.* | *Das Gelenk liegt unter dem Joint-Scope der SO-101-Articulation.* |

[Bearbeitete und erneut geöffnete Szenenvariante](docs/images/tutorial/17-isaac-scene-edited.png)

`.usda` speichert lesbaren USD-Text. `.usdc` speichert das binäre Crate-Format. `.usd` kann eines dieser Formate enthalten. Die Stage referenziert `robot.usd` und weitere Robot-Layer. Für einen Umzug den ganzen Szenenordner samt Referenzen übernehmen, bei absoluten USD-Referenzen die Pfade in Isaac neu zuordnen und den neuen Stand erneut in Studio laden.

[Isaac-Sim-Adapter und Physik](docs/ISAAC_SIM_INTEGRATION.md) · [NVIDIA URDF-Importer](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/importer_exporter/import_urdf.html)

### 9. ROS-Bridge und Action Graph prüfen

Der SDK-Prozess erstellt unter `/World/ROS2` den Graphen mit der Domain und den Topics des gewählten Profils. In der Standardkonfiguration veröffentlicht er `/joint_states` und `/clock`, empfängt `/joint_command` und übergibt Namen, Positionen und Geschwindigkeiten an den Articulation Controller. ROS-2-JointStates verwenden Radiant bzw. Meter. Der Studio-Adapter setzt den Trajektorienplan in diese Kommandos um und prüft das Ziel anhand tatsächlicher Rückmeldungen.

```mermaid
flowchart LR
    Tick["On Playback Tick"] --> States["ROS2 Publish Joint State"]
    Tick --> Commands["ROS2 Subscribe Joint State"]
    Tick --> Clock["ROS2 Publish Clock"]
    Context["ROS2 Context · Domain 1"] --> States
    Context --> Commands
    Context --> Clock
    SimTime["Read Simulation Time"] --> States
    SimTime --> Clock
    Commands --> Controller["Articulation Controller"]
    Controller --> Arm["SO-101 Articulation"]
    Arm --> States
```

*Schema des erzeugten Action Graphs.*

In Isaac **Window → Visual Scripting → Action Graph** öffnen und den Graphen **ROS2** laden. Im Stage Tree `/World/ROS2/Context` auswählen, um die Domain unter **Property** zu kontrollieren.

![Tatsächlich erzeugter ROS-Action-Graph in Isaac Sim](docs/images/tutorial/15-isaac-ros-graph.png)

*Die Publisher, der Subscriber und der Articulation Controller verwenden die im Graph sichtbaren Verbindungen.*

![ROS2 Context mit domain_id 1 in der Isaac-Property-Ansicht](docs/images/tutorial/16-isaac-ros-context.png)

*Die Domain kommt aus dem Studio-Profil. Use Domain ID Env Var ist für diesen Graph abgewählt.*

Der Workflow erzeugt den Graphen für eine neue Szene und richtet ihn beim Start einer kompatiblen eigenen USD in einer Arbeitskopie erneut ein. Für externe Simulatoren mit eigener Bridge die vorhandenen Topics, Domain und Controller im Backendprofil zuordnen.

[Isaac-Sim ROS Joint Control](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_manipulation.html) · [Studio ROS-Ausführung](docs/ROS_EXECUTION.md)

### 10. Gespeicherte Szene erneut starten

1. Laufende Studio-Simulation mit **Disconnect** vom ROS-Backend trennen und unter **Backends… → Stop owned Isaac process** den von Studio gestarteten Prozess beenden.
2. Im Runtime-Banner das Isaac-Profil auswählen und über **Browse…** neben **Simulation File** die eigene USD öffnen. Der Dateiname erscheint danach in der Auswahl und unter **Recent…**.
3. **Start Simulation** klicken. Im Banner auf **Loaded & validated** und **Bridge: Connected** warten. Bei manueller Steuerung bleibt Isaac zunächst pausiert. Im Isaac-Fenster **Play** klicken.
4. **Simulation Ready** bestätigt nutzbare Simulationszeit und Gelenkfeedback. **Auto Play** ist die explizite Alternative, um den eigenen Prozess direkt in die Wiedergabe zu versetzen.

![Isaac-Banner mit validierter Szene und wartender Wiedergabe](docs/images/tutorial/19-simulation-paused.png)

*Vor Play zeigt der Banner den pausierten Zustand.*

![Isaac-Banner mit Simulation Ready](docs/images/tutorial/20-simulation-ready.png)

*Nach Play meldet das Banner Gelenkzustand und Ausführungsbereitschaft.*

### 11. Gelenkbewegung und Live Monitoring

Bei einer aktiven Isaac-Simulation folgen **Robot Configuration** und **Live Monitoring** den empfangenen ROS-Gelenkwerten. Nach **Play** und **Simulation Ready** senden die Regler im Studio Positionsziele über den konfigurierten Simulationscontroller. Alternativ unter **Robot Configuration → Positions & Tasks → Task editor** einen Gelenkschritt anlegen. Das Beispiel in Schritt 13 bewegt `shoulder_pan` um ungefähr `5.73°` aus der Nullpose.

**Live Monitoring** öffnen und die Position sowie Geschwindigkeit des Gelenks auswählen. Ein gesendetes Ziel und die zurückgemeldete Position sind separate Werte. Der Task meldet Erfolg, wenn die beobachtete Gelenkstellung das Ziel innerhalb der konfigurierten Toleranz erreicht. Den Tab an den rechten Fensterrand von **Robot Configuration** ziehen, um beide Panels zu sehen. **View → Save workspace layout** speichert das Layout. **Restore saved workspace layout** stellt es wieder her.

![Gelenkrückmeldung während eines Isaac-Tasks im geteilten Workspace](docs/images/tutorial/22-live-monitoring.png)

*Das Monitoring folgt der laufenden Simulationsbewegung. ROS-Verbindungen zu physischen Robotern sind lesend.*

### 12. Positionen speichern

1. Für die Offline-Übung **Offline demo** wählen und den Arm in eine gültige Pose bringen. Unter **Robot Configuration → Positions & Tasks → Teach-in positions** auf **Capture joints** klicken.
2. Den Namen `Home Lab` vergeben und **Save** wählen. Für eine Position mit XYZ und Werkzeugframe **Capture Cartesian** verwenden. Auf dem SO-101 für das erste Ziel den Modus **Position Only** wählen.
3. Einen Eintrag in der Tabelle auswählen. **Recall / apply offline** ruft ihn im virtuellen Modell ab. **Check reachability** prüft die Erreichbarkeit für die bestätigte Kette. Beim Wechsel zurück zu Isaac kann eine gespeicherte Position im Taskeditor als Ziel dienen.

![Teach-in positions mit Aktionen zum Erfassen und Abrufen](docs/images/tutorial/23-teach-in.png)

*Teach-in führt Gelenk- und kartesische Positionen in derselben Projektbibliothek.*

### 13. Eigenen Task erstellen und ausführen

1. **Positions & Tasks → Task editor → New** öffnen und den Namen `SO-101 shoulder check` eingeben.
2. **Add** klicken. Unter **Action → Move to Joint Position** und **Target source → Direct input** alle Achsen in Nullstellung lassen, `shoulder_pan` auf **5,73°** und **Minimum move time / wait** auf **1,5 s** setzen. **Save** klicken.
3. Einen zweiten Schritt über **Add → Wait** mit **1 s** speichern.
4. Einen dritten Schritt über **Add → Home**, **Target source → Default virtual home** und **1,5 s** speichern. Alternativ die passende gespeicherte Gelenkposition wählen.
5. **Validate all** klicken. In der Offline-Ansicht **Preview** wählen. Mit aktivem und bereitem Isaac-Backend sendet derselbe **Preview**-Knopf den validierten Task an die Simulation. **Live Monitoring** zeigt Gelenkrückmeldung und Abschluss. Über **Save** das Projekt sichern und über **Open…** erneut öffnen.

| Weitere Aktion | Eingabe im Schrittdialog |
| --- | --- |
| **Move to Cartesian Pose** | Unter **Target source** eine gespeicherte Cartesian-Position oder **Direct input** wählen. Bei Direkteingabe XYZ in mm, den Zielrahmen und den IK-Modus setzen. |
| **Move Relative · endpoint IK** | **Relative reference frame** auf `base_link` setzen und einen kleinen Versatz wie `dZ = 5 mm` bei unveränderter Orientierung prüfen. Die Bewegung fährt eine Gelenkraumkurve zum gelösten Endpunkt. |
| **Open Gripper / Close Gripper** | Den modellgültigen Greiferwinkel und die Mindestdauer setzen. Die virtuellen Attach-/Release-Optionen gehören zur Offline-Vorschau. |

![Schrittdialog mit direkter Schulter-Gelenkposition](docs/images/tutorial/25-task-action.png)

*Der Schrittdialog zeigt die direkte Gelenkeingabe für Turn shoulder. Save validiert das Ziel.*

![Bestätigter SO-101-Testtask mit drei Schritten](docs/images/tutorial/26-task-completed.png)

*Die dokumentierte Abfolge prüft Gelenkziel, Wartezeit und die Rückkehr zur Nullpose.*

`./start.sh --pick-place-demo` öffnet zusätzlich das Beispiel mit 13 Schritten und virtueller Objektzuordnung. Die Abnahme bestätigt Gelenk- und Greiferbewegungen. Physischer Objektkontakt ist ein eigener, noch ausstehender Test.

### Stereo, Qwen und MoveIt

Ein separat ausführbarer, tatsächlich trainierter Stereo-Tiefenschätzer mit Isaac-Datengenerator,
OpenCV-Vergleich, ONNX-Export und optionalem Studio-Adapter ist in der
[Stereo-Trainingsdokumentation](docs/STEREO_DEPTH_TRAINING.md) beschrieben. Die Messwerte gelten für Simulation.
Zusätzliche [milde Blur-/Fisheye-Tests](docs/stereo/robustness-mild/RESULTS.md) prüfen günstige Kameraoptik und kleine Kalibrierabweichungen auf identischen Testszenen.
Die beiden [getrennten Höhenmodelle mit extremer Fisheye-Optik](docs/STEREO_HEIGHT_FISHEYE.md) ersetzen den bisherigen Stereo-Modellstand: [42–60 cm](checkpoints/so101-high-height-fisheye/README.md) und [10–20 cm](checkpoints/so101-low-height-fisheye/README.md) senkrechte Montagehöhe bei 60 mm paralleler Basis und 0–45° gegen die Tischfläche. Beide geprüften PyTorch-/ONNX-Modelle, Konfigurationen, Messberichte und kalibrierte Rohbildbeispiele sind enthalten; die Inferenz benötigt keinen laufenden Simulator. Umfangreiche Datensätze und fortsetzbare Trainingsstände bleiben lokal.

Der neue Reiter **Camera & Vision → Stereo & Planning** koppelt zwei ROS-Kameras mit einer Arbeitsszene, lokalem Ollama und MoveIt Plan Only. Isaac Sim kann dazu auf einem separaten Rechner laufen. [Einrichtung und Abnahme](docs/ISAAC_SIM_RGB_CAMERA.md#stereo-und-qwen-mit-einem-separaten-isaac-rechner)

### 14. RGB-Kamera zur USD hinzufügen

1. Die eigene Szene auswählen und den laufenden eigenen Simulator stoppen.
2. **Sensor Configuration → Add Camera to Isaac Sim…** öffnen. Im Assistenten einen Kameranamen und das Isaac-Backend wählen. **Mounting → World fixed** verwenden.
3. Für das Beispiel die Position in Weltmetern `(0.23, -0.18, 0.68)` und **Look at world point (m)** `(0.23, -0.18, 0.0)` eintragen. **Apply look-at** klicken. **Render resolution** `640 × 480`, nominale **Render FPS** `20`, horizontales FOV `70°`.
4. RGB `/so101/camera/image_raw` und CameraInfo `/so101/camera/camera_info` prüfen. Für das Tutorial **Add red test cube to the Studio mounting table** aktivieren. **Create camera…** klicken und einen neuen USD-Dateinamen wählen.
5. Studio speichert eine abgeleitete USD mit Kamera-Prim, Render Product, Publishern und statischen Kamera-TFs. Diese Szene wird im Banner ausgewählt. Für das Projekt **Save** klicken.

![Isaac-RGB-Kamera-Assistent mit World fixed und Look-at](docs/images/tutorial/27-camera-setup.png)

*Der Assistent zeigt die Kameraausrichtung und die ROS-Topics vor dem Erzeugen der abgeleiteten USD.*

![Isaac-Stage mit StudioCameras, Testwürfel und StudioVisionROS](docs/images/tutorial/28-isaac-camera-stage.png)

*Der Stage Tree enthält den Kamerazweig und den separaten ROS-Vision-Graph.*

### 15. RGB-Stream anzeigen

**Start Simulation** und in Isaac **Play** klicken. Unter **Camera & Vision** die Kamera auswählen und **Start Stream** drücken. Die Anzeige meldet Auflösung, Bildrate, ersetzte Frames und den letzten ROS-Zeitstempel. **Snapshot…** speichert den aktuellen RGB-Frame als PNG. Für eine externe ROS-Kamera in **Sensor Configuration → Add Camera…** das zugehörige Backend und den Bildtopic auswählen.

![Tatsächliches RGB-Kamerabild aus Isaac Sim](docs/images/tutorial/28-rgb-stream.png)

*Das Livebild stammt vom Render Product der platzierten Kamera.*

### 16. Detektor auswählen

![Detektor-Dropdown in Camera & Vision](docs/images/tutorial/29-detector-dropdown.png)

*Das Dropdown führt die implementierten Detektoren und den vorbereiteten Custom-Eintrag auf.*

- **Color / HSV Detection** wählen und **Settings…** öffnen. Die zwei Standardintervalle für Rot und die Mindestfläche von 100 Pixeln eignen sich für den Testwürfel. **Start Detection** klicken und einen Eintrag in der Objektliste wählen.
- **ArUco Marker Detection** verwendet standardmäßig `DICT_4X4_50`. Das Wörterbuch unter **Settings…** passend zum Marker wählen. Ein sichtbarer Marker liefert ID und vier Bildecken.
- **YOLO ONNX** verwendet ein separat bezogenes kompatibles Modell. **Add ONNX…** öffnen und ein lokales YOLOv5n-v7-COCO-Modell mit festem 640er-Eingang auswählen. Unter **Settings…** den Modellscore-Schwellwert setzen, beispielsweise `0.25`. **Start Detection** startet die CPU-Inferenz und prüft den Modellvertrag. Die Ergebnisliste zeigt Klassen, Scores und Boxen.
- **Disabled** zeigt weiter RGB. **Stop Detection** beendet den Detektor und **Stop Stream** den Kameraempfang.

![HSV-Intervalle, Mindestfläche und Inferenzgrenze im Einstellungsdialog](docs/images/tutorial/29-hsv-settings.png)

*Für Rot verwenden die Standardwerte zwei Hue-Intervalle an den Enden der Skala.*

![HSV-Overlay und Objektliste für den roten Würfel](docs/images/tutorial/30-detection-result.png)

*Die HSV-Erkennung markiert den Würfel mit Box und Referenzpixel. Der angezeigte Wert ist der Maskenfüllgrad.*

![ArUco-Marker 17 im ROS-Kamerastream](docs/images/tutorial/32-aruco.png)

*ArUco zeigt die ID und die Bildecken eines für den Test erzeugten Markers.*

![YOLO ONNX erkennt ein synthetisches Stoppschild in einem ROS-Teststream](docs/images/tutorial/33-yolo-onnx.png)

*YOLOv5n v7 erkennt die Klasse stop sign mit Score 0,917. Die eigens gezeichnete Tafel ist der Inhalt eines ROS-Teststreams.*

[Detektor-Einstellungen](docs/OBJECT_DETECTION.md) · [Geprüfter YOLO-Modellvertrag](docs/YOLO_ONNX_INTEGRATION.md) · [Vision-Messdaten](docs/VISION_VERIFICATION.md)

### 17. Objektposition bestimmen und Projekt erneut öffnen

1. Bei laufendem Stream **Calibrate…** öffnen. Die Live-CameraInfo enthält Brennweiten, Hauptpunkt und Verzerrung. TF2 liefert die Kamerapose zum Bildzeitpunkt. In der Demo stammt die Arbeitsebene aus dem Montagetisch mit Welt-`Z ≈ -0.005 m`.
2. Bei einer eigenen Szene Punkt und Normale der gemessenen Arbeitsebene eingeben und die Bestätigung aktivieren. Optional **Store the current CameraInfo and TF2 calibration record** wählen, speichern und den Stream neu starten.
3. In der Objektliste den Würfel auswählen. Unter **Reference pixel** **Bounding-box center** wählen und **Output frame → Robot base** setzen. Studio schneidet den Kamerastrahl mit der Ebene und transformiert den Schnittpunkt nach `base_link`.
4. **Use as Target Draft…** kann einen geprüften Punkt mit eingegebener Werkzeugorientierung und Versatz in einen manuellen Entwurf übernehmen. Das Ergebnis ist eine planare Positionsschätzung. Homographie und automatisches Greifen stehen aus.
5. **Save** speichert Aufgaben, Positionen, Backend und Kamera im Projekt. Die lokalen USD-/Installationspfade werden auf einem anderen Rechner über **Browse…** und **Backends… → Edit** neu zugeordnet.

![Kalibrierungsdialog mit gültiger CameraInfo und bestätigter Arbeitsebene](docs/images/tutorial/31-calibration.png)

*Die gespeicherte Kalibrierungsaufnahme dokumentiert die gemessenen Daten. Live-Schätzungen verwenden die zum Bild passenden Laufzeitwerte.*

### Eigene Roboter und Szenen

1. **New project** wählen und über **Import URDF / Xacro** eine eigene URDF mit erreichbaren Mesh-Dateien laden. Linkbaum, Joint-Typen, Achsen und Grenzwerte im Viewer kontrollieren.
2. Unter **Chain setup** Basis, Werkzeug, optionale Greiferachse und Solver setzen. Die FK-/IK-Prüfung ausführen und erreichbare Ziele anhand der Freiheitsgrade des Modells bestimmen.
3. Für Isaac eine neue Stage aus dem gebundenen Modell erzeugen oder eine kompatible USD über **Browse…** zuordnen. Die automatische Validierung erwartet Meter, Z-up, eine Articulation unter `/World/Robot` und zur URDF passende Gelenke, Achsen und Limits. Eine externe ROS-Simulation kann ihre eigenen Controller und Topics bereitstellen.
4. Im Backendprofil Domain, Controller-Schnittstelle, Topics und Mapping prüfen. JointState-Feedback vor einer Taskausführung im Monitoring bestätigen. Einen kleinen Task validieren und das Projekt speichern.
5. Für die Offline-Einführung gibt es das [generische Beispiel](examples/generic_robot/demo.studio.yaml) und das [portable SO-101-Katalogprojekt](examples/so101/so101.studio.yaml). USD-Dateien werden über Schritt 8 lokal erzeugt.

## Projekte und Isaac-Szenen öffnen

Studio speichert Robotereinstellungen und Backendprofile gemeinsam in der jeweiligen `.studio.yaml`. Öffne sie über **Project → Open…**. Die gespeicherten Backends findest du danach unter **Simulation Runtime** und **Backends…**. Der Ordner [Projects/](Projects/) ist im Checkout der vorgeschlagene Speicherort für neue Projektdateien. Er kann auch Isaac-Sim-Dateien mit ihren zugehörigen USD-Layern aufnehmen. Bei einem dort gespeicherten Projekt legt Studio generierte Szenen standardmäßig im Unterordner `isaac-scenes/` neben der Projektdatei ab.

## Physischen SO-101 Follower über USB anbinden

Studio liest die sechs Motorpositionen des SO-101 Followers über den USB-Seriell-Adapter. Das Projekt [SO-101-Real.studio.yaml](Projects/SO-101-Real.studio.yaml) bietet manuelle Einzelgelenksteuerung. Die USB-Runtime liefert ihre Rückmeldungen an **Robot Configuration** und **Live Monitoring**. Studio-Tasks laufen über den Offline- oder den ROS-Simulationscontroller.

### LeRobot installieren und den Follower kalibrieren

1. LeRobot in einer eigenen Python-Umgebung einrichten und die Feetech-Unterstützung in derselben Umgebung installieren. Bei einer Installation von PyPI lautet der Befehl `pip install 'lerobot[feetech]'`. Im LeRobot-Quellcodeordner lautet er `pip install -e '.[feetech]'`. [Offizielle LeRobot-Installation](https://huggingface.co/docs/lerobot/installation) · [Feetech-Unterstützung](https://huggingface.co/docs/lerobot/installation#motor-control)
2. Den Follower-Adapter per USB anschließen und den Arm mit Strom versorgen. In der LeRobot-Umgebung `lerobot-find-port` starten und den Follower-Adapter abziehen, wenn das Programm dazu auffordert. Den gemeldeten Port merken, den Adapter wieder anschließen und den [USB-Zugriff prüfen](#usb-zugriff-unter-ubuntu-prüfen). Wenn die Motor-IDs oder die Baudrate noch eingerichtet werden müssen, den Schritten für den Follower folgen. [USB-Port und Motoren einrichten](https://huggingface.co/docs/lerobot/so101#configure-the-motors)
3. Den Follower nach der Originalanleitung kalibrieren, falls für diesen Arm noch keine Kalibrierung vorliegt. Im Beispiel heißt der Arm `SO101` und der Anschluss `/dev/ttyACM0`. Ersetze den Port durch den zuvor ermittelten Anschluss. LeRobot speichert die Kalibrierung unter der gewählten Roboter-ID. [Offizielle SO-101-Kalibrierung mit Video](https://huggingface.co/docs/lerobot/so101#calibrate)

   ```bash
   lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=SO101
   ```

### USB-Zugriff unter Ubuntu prüfen

Wenn Studio **No access** meldet, prüfe mit `ls -l /dev/ttyACM0` die Gerätegruppe und mit `id -nG` die Gruppen der aktuellen Sitzung. Gehört der Anschluss zu `dialout` und fehlt diese Gruppe in deiner Sitzung, füge deinen Benutzer hinzu. Melde dich anschließend vollständig ab und wieder an und starte Studio aus der neuen Sitzung.

```bash
sudo usermod -aG dialout "$USER"
```

### Port auswählen und Gelenkpositionen lesen

1. Im Runtime-Banner **Simulation Runtime → Real Robot** wählen. **Robot Connection** erscheint und listet die USB-Seriell-Adapter auf. Wähle den Port des Followers. **Refresh ports** lädt die Liste nach dem Einstecken neu. Unter Linux verwendet Studio, sofern vorhanden, intern den stabilen Gerätenamen unter `/dev/serial/by-id/`.

   ![Runtime-Banner mit Real Robot und ausgewähltem USB-Adapter ttyACM0](docs/images/hardware/usb-runtime.png)

2. **Read USB Joints…** öffnen. Studio verwendet den gespeicherten LeRobot-Python-Pfad, falls einer hinterlegt ist. Sonst findet es eine übliche Miniforge-Installation mit dem Environment `lerobot` automatisch. Bei einer anderen Conda-Installation Studio aus dem aktivierten `lerobot`-Environment starten.
3. Unter **LeRobot calibration** erscheint eine einzelne gefundene Follower-Datei automatisch. Studio sucht im LeRobot-Cache und berücksichtigt `HF_LEROBOT_CALIBRATION`. Mit **Choose calibration file…** kannst du die JSON-Datei aus einem anderen Ordner auswählen. Schließe LeLab oder andere Programme, die denselben seriellen Port verwenden.
4. **Live USB · calibrated · torque off** bestätigt eine passende Kalibrierung. Studio zeigt fünf Armwinkel in Grad und den normierten Greiferwert in Prozent. Ohne passende Kalibrierung erscheinen rohe Motorzählwerte. Bei ausbleibender Antwort prüfe den Port, die Stromversorgung und die Motor-Einrichtung.

   ![USB-Dialog mit ausgewählter SO-101-Kalibrierung und sechs gemessenen Gelenkpositionen](docs/images/hardware/usb-joint-positions.png)

5. **Close** blendet den USB-Dialog aus. Die Messwerte werden weiterhin in **Robot Configuration** und **Live Monitoring** aktualisiert, sobald die Kalibrierung passt. **Disconnect** im Runtime-Banner beendet die USB-Verbindung.

### Einzelne Gelenke am echten Arm bewegen

1. **Project → Open…** wählen und `Projects/SO-101-Real.studio.yaml` öffnen. Das Projekt verwendet das SO-101-Referenzmodell aus der Offline-Vorlage. Den USB-Port des Followers unter **Robot Connection** wählen und **USB Joint Control…** öffnen.
2. Den Arm am Tisch befestigen und so abstützen, dass er sich auch bei ausgeschaltetem Drehmoment nicht absenkt. Den Bewegungsbereich freihalten und die Stromversorgung erreichbar halten.
3. Im USB-Dialog die Kalibrierungsdatei prüfen und warten, bis die gemessenen Ausgangspositionen angezeigt werden. Der Dialog kann danach geschlossen oder neben dem Hauptfenster geöffnet bleiben.
4. In **Robot Configuration** unter den Slidern das Kontrollkästchen **Arm is fixed and supported. Enable motor position control.** setzen. Sobald der Controller die Freigabe bestätigt, werden die Slider bedienbar. Die sechs Motoren übernehmen zuerst ihre gemessene Position und halten sie.
5. Einen Slider oder Zahlenwert in **Robot Configuration** ändern. Studio sendet das Ziel direkt an den jeweiligen Motor. Die 3D-Ansicht und **Live Monitoring** folgen den gemessenen Werten. Ziele bleiben innerhalb der kalibrierten und der URDF-Grenzen. Bei Isaac Sim und MuJoCo steuern dieselben Regler die jeweils aktive Simulation.
6. Die Freigabe bleibt beim Ausblenden des USB-Dialogs aktiv. Stütze den Arm ab, bevor du den Haken entfernst oder **Disconnect** klickst. Beides schaltet das Motordrehmoment ab. Wenn Studio meldet, dass das Abschalten nicht bestätigt werden konnte, lass den Arm abgestützt und trenne die Motorversorgung.

### Geschwindigkeit und maximales Drehmoment einstellen

Unter der Motorfreigabe in **Robot Configuration** stehen zwei Regler für alle sechs SO-101-Motoren zur Verfügung, sobald die USB-Kalibrierung bestätigt ist.

- **USB Speed** (blau, runder Griff) setzt die Sollgeschwindigkeit auf 1–240 Encoder-Schritte pro Sekunde. Eine neue USB-Verbindung startet mit 24 Schritten pro Sekunde.
- **USB Max torque** (orange, eckiger Griff) setzt eine Obergrenze von 1–100 % der Servo-Skala. Der Standardwert von 100 % behält die vorhandenen Motorgrenzen bei. Wenn ein Motor tiefer begrenzt ist, etwa der Greifer, gilt seine niedrigere Grenze. Bei einem niedrigen Wert kann der Arm seine Position unter Last verlieren.

Du kannst beide Werte vor der Motorfreigabe einstellen. Bei aktiver Freigabe übernimmt Studio eine Änderung, sobald du den Regler loslässt. Beim Deaktivieren der Motorfreigabe oder Trennen der Verbindung stellt Studio die zuvor gelesenen Registerwerte wieder her.

### SO-101 von Hand führen, aufnehmen und wiedergeben

1. Verbinde den kalibrierten SO-101 per USB und öffne **Robot Configuration → Positions & Tasks → Teach-in positions**.
2. Wähle vor **Record** unter **Recording torque** eine Drehmomentgrenze zwischen 0 und 25 %. Der Standardwert ist 10 %. Aktiviere für eine Aufnahme mit Haltemoment die Motorfreigabe. Bei **0 %** sind die Motoren drehmomentfrei und Studio kann auch ohne Motorfreigabe aufnehmen. Stütze den Arm ab und halte Abstand zu Quetschstellen.
3. Gib unter **Recording title** den Dateinamen ein und klicke **Record**. Bei aktivem Haltemoment übernimmt Studio gemessene Positionsänderungen beim Führen als neue Halteziele. Der USB-Prozess zielt dabei auf bis zu 100 Messungen pro Sekunde. Die tatsächliche Rate hängt vom Bus ab. Auch ein Absacken unter Last verändert die Messwerte.
4. **Stop recording** schließt die CSV in `Replays/<Titel>.csv` ab und wählt sie im Dropdown aus. Die Datei enthält Zeit, fünf Gelenkwinkel, die Greiferstellung und die sechs Encoderpositionen aus jeder Messung. Nach einer Aufnahme mit Haltemoment hält der Arm die aktuelle Pose mit den normalen USB-Grenzen. Nach einer Aufnahme bei 0 % bleibt das Motordrehmoment ausgeschaltet.
5. Aktiviere für die Wiedergabe die Motorfreigabe, wähle die CSV und klicke **Replay**, nachdem du den Bewegungsbereich freigegeben hast. Studio verwendet **USB Speed** und **USB Max torque** aus **Robot Configuration** bereits für die Fahrt zur Startpose. Es verlängert Übergänge, die mit der gewählten Geschwindigkeit länger dauern. Die Aufnahme braucht die ursprüngliche Motorkalibrierung.
6. Beim Replay korrigiert ein äußerer PID-Regler die Positionsziele anhand der gemessenen Encoderwerte. Der Sollwert darf je Motor um bis zu 2° während der Fahrt und bis zu 4° beim Anfahren der Start- oder Endpose korrigiert werden. Die kalibrierten Bereiche und die USB-Grenzen für Drehmoment und Geschwindigkeit gelten weiter. Die Start- und Endpose gelten bei einer gemessenen Abweichung von höchstens 2° pro Motor als erreicht. Kommt der Arm nicht zur Pose, hält Studio die aktuelle Position und meldet die größte Abweichung. **Stop replay** hält ebenfalls die aktuelle Pose.

### USB-Kamerabilder am SO-101

Das Profil `Projects/SO-101-Real.studio.yaml` enthält zwei USB-Kameras. Unter **Camera & Vision → RGB** eine Kamera wählen und **Start Stream** klicken. Beim Wechsel der Kamera den Stream erneut starten. Die Vorschau benötigt OpenCV aus dem Studio-Extra `vision`. Falls die Kameras an andere USB-Ports gesteckt werden, lassen sich ihre Gerätepfade unter **Sensor Configuration → Configure… → Local USB device (optional)** aktualisieren.

Für ROS 2 die direkte Vorschau mit **Stop Stream** schließen. Wähle im Projektordner das [Höhenmodell](docs/STEREO_HEIGHT_FISHEYE.md) passend zum senkrechten Abstand zwischen Kamera und Arbeitsfläche.

```bash
SO101_STEREO_PROFILE=high bash scripts/start_so101_usb_cameras.sh
```

`high` ist für 42–60 cm vorgesehen und `low` für 10–20 cm. Das Skript verwendet das Studio-Humble-Image als Basis und baut beim ersten Start ein separates Kamera-Image. Optional lassen sich die beiden V4L2-Gerätepfade in der Reihenfolge links, rechts als Argumente übergeben. Der Treiber publiziert in ROS-Domain 23 über Loopback. `ROS_DOMAIN_ID` kann beim Start überschrieben werden. **Ctrl+C** beendet den Treiber.

| Ansicht | ROS-Topic | Bildformat |
|---|---|---|
| Kamera 1 | `/so101/usb/camera_1/image_raw` | `rgb8`, 640 × 480 |
| Kamera 2 | `/so101/usb/camera_2/image_raw` | `rgb8`, 640 × 480 |
| Nebeneinander | `/so101/usb/stereo/image_raw` | `rgb8`, 1280 × 480 |
| Relative Tiefenkarte | `/so101/usb/stereo/depth_relative` | `mono8`, 640 × 480 |

Alle vier ROS-Ansichten gehören zum Profil **Real Robot · SO-101**. **USB Joint Control…** startet bei verfügbarer Humble-Runtime auch die ROS-Kamerabrücke. **Connect Cameras** verbindet die Brücke bei ausgewähltem USB-Port auch separat. Unter **Camera & Vision → RGB** eine der vier ROS-Ansichten wählen und **Start Stream** klicken. **Disconnect** im Runtime-Banner beendet die Studio-Verbindungen für USB-Gelenke und Kamerabrücke.

Die Stereoansichten verwenden Aufnahmen mit maximal 25 ms Abstand zwischen den Empfangszeitstempeln am PC. Die trainierte CompactStereo-Policy berechnet direkt aus den USB-Bildern die Disparität. Die Tiefenansicht zeigt deren Kehrwert als Graustufenbild. Schwarz markiert ungültige Pixel. Hellere gültige Pixel stehen für größere relative Tiefe. Die Werte haben keine metrische Einheit. `SO101_STEREO_MODEL` erlaubt es, einen anderen kompatiblen ONNX-Export zu laden.

Mit verbundenem Profil **Real Robot · SO-101** öffnet **Camera & Vision → Stereo & Planning → Start Stereo** das Bildpaar und die zugehörige Policy-Tiefenkarte. Studio ordnet beide anhand des identischen Zeitstempels zu. **Check Ollama** prüft das Modell `qwen3-vl:8b-instruct`.

Gib eine Beschreibung wie „grüner Würfel mit schwarzem Aufkleber“ ein und klicke **Find Objects**. Qwen sucht die Treffer in Kamera 1. Studio vergibt IDs wie `green_cube_1` und markiert die Treffer unter **Detected Objects**. Die IDs gelten für diese Aufnahme. **Interpret Task** führt dieselbe Erkennung automatisch aus und verwendet die gefundenen IDs, das Stereo-Bildpaar und die relative Tiefenkarte für den Auftragsvorschlag. **Plan Only** benötigt metrische Objektpositionen im Roboterkoordinatensystem und bleibt bei relativer USB-Tiefe gesperrt.

## Lokale MuJoCo-Runtime

Die optionale Abhängigkeit im Studio-Projektordner installieren.

```bash
.venv/bin/python -m pip install '.[mujoco]'
```

1. Eine MuJoCo-kompatible URDF in Studio laden. Unter **Simulation Runtime → MuJoCo** ein Profil auswählen oder anlegen.
2. Unter **Simulation File** **Generate from current robot** lassen oder über **Browse…** eine eigene MJCF-Szene auswählen. Bei einer Datei müssen die skalaren Gelenke und die Zuordnung zum Studio-Modell passen. Die Regler steuern passende Positionsaktoren mit direkter Gelenkübertragung. Beim Start aus der URDF erzeugt Studio diese Aktoren.
3. **Start Simulation** klicken. MuJoCo läuft lokal, und die gemessenen Positionen und Geschwindigkeiten erscheinen in **Robot Configuration** und **Live Monitoring**. Die Regler setzen Ziele in der Simulation. Mit **Disconnect** zur Offline-Ansicht zurückkehren.

Bei rein kinematischen Studio-Demos ohne Massenträgheitsangaben verwendet der lokale URDF-Import Mindestwerte von 0,1 kg und 0,001 kg·m², damit MuJoCo sie starten kann. Für physikalisch vergleichbare eigene Szenen sollten die URDF oder MJCF die tatsächlichen Massen und Trägheiten enthalten.

[Installieren](INSTALLATION.md) · [Entwickeln und testen](DEVELOPMENT.md) · [Mitwirken](CONTRIBUTING.md) · [Projektformat](docs/PROJECT_FORMAT.md) · [Roadmap](ROADMAP.md) · [Prüfbericht](docs/PUBLICATION_VERIFICATION.md) · [Apache-2.0](LICENSE) · [Drittanbieter-Lizenzen](THIRD_PARTY_LICENSES.md)
