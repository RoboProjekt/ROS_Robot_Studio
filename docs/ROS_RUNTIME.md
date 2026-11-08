# Docker ROS Runtime

Start: `./start.sh --pick-place-demo`. Im vorhandenen **Project & Runtime** die ROS-Version
wählen und **Open Live ROS Demo** ausführen. Alternativ nutzt die vorhandene Werkzeugleiste
**Start ROS demo** denselben Ablauf. **Offline demo** bzw. **Disconnect ROS → Offline**
führt kontrolliert zurück. Die Backendauswahl im Taskeditor ist daran gekoppelt.

## Installation und Isolation

| Runtime | Offizielle Basis | Container-Python |
| --- | --- | --- |
| Humble | `ros:humble-ros-base-jammy` | Ubuntu 22.04 |
| Jazzy | `ros:jazzy-ros-base-noble` | Ubuntu 24.04 |
| Lyrical | `ros:lyrical-ros-base-resolute` | Ubuntu 26.04 |

Der Installer prüft Registry-v2-Manifeste für amd64, pinnt den Basisdigest und baut
`ros-robot-studio/<distro>:0.1.0`. Gemeinsame Ressourcen liegen im bestehenden
`data/runtime`-Buildkontext. Paketquellen werden signiert geprüft; die Paketversionen
werden in `/opt/studio-packages.txt` erfasst. Apt ist kein eingefrorener Snapshot.

Installiert werden rosbridge/rosapi, rclpy und ROS-Basisinterfaces, control_msgs,
trajectory_msgs, tf2_ros/tf2_msgs, diagnostic_msgs, Fast DDS, xacro sowie NumPy/defusedxml
im Container. Es gibt keinen rclpy-Import im Desktop. Die originale ausgewählte URDF wird
als Teil eines begrenzten JSON-Modellvertrags in den eigenen Container kopiert; keine
Host-Mounts, USB-Geräte, Docker-Socket-Mounts oder privilegierten Container.

Eigene Images/Container tragen `io.ros-robot-studio.managed=0.1.0`. `studio.runtime_source`
ist ein SHA-256-Fingerabdruck aller enthaltenen Studio-Runtimequellen. Veraltete Images
werden nicht als aktuell installiert gewertet und müssen neu gebaut werden. Ein Build
allein bedeutet **nicht Verified**. Der Status **Verified · native ROS action + feedback**
erfordert einen tatsächlich abgeschlossenen Diagnoselauf und die passende konkrete Image-ID.

Container laufen als Benutzer `studio`, mit verworfenen Capabilities, no-new-privileges,
CPU-/RAM-/PID-Grenzen und begrenzten Logs. Jede verwaltete Bridge-Runtime erhält ein eigenes gekennzeichnetes Docker-Netz, das beim
Stoppen entfernt wird. Gleichnamige ROS-Topics verschiedener Sitzungen bleiben getrennt.
Standardmäßig wird nur rosbridge an `127.0.0.1:<port>` veröffentlicht. Stop/Restart/Logs/Remove prüfen Ownership. Fremde
Container werden nicht angefasst. Images werden niemals mit Force entfernt.

## Moduswechsel

Ein Start prüft Docker/Image, bietet die fehlende Installation an und stoppt die lokale
Taskwiedergabe kontrolliert. Erst nach erfolgreichem Containerstart werden Transport,
State-Lease und Ausführungsbackend gewechselt. Frische JointStates und eine echte
Controlleranzeige sind zusätzlich erforderlich. Ein verbundener Graph ist keine Ausführungsfreigabe.

Fehlender Dockerzugriff öffnet eine klare Diagnose/Setup-Möglichkeit. Ein Startfehler
bewahrt den Offline-Modus; fehlendes State-/Controllerfeedback beim ROS-Anlauf führt
nach 20 Sekunden zurück. Während ROS aktiv ist, verändern manuelle Gelenkregler keine
Rückmeldungen. Disconnect beendet Actionbefehle, Subscriptions, alte Quellen und den
eigenen Container, aktiviert Offline und behält die letzte virtuelle Pose.

## DDS und externe Simulation

**DDS / Simulation Settings** speichert Domain-ID (Standard 42), Fast DDS, Namespace,
Discovery, UDP-Profil, Bridge-/explizites Hostnetzwerk, Loopback-Port sowie Simulationstyp,
Topics, Action und Joint-Mapping. Änderungen erfordern zuvor Disconnect.

Hostnetzwerk erfordert eine ausdrückliche Bestätigung des externen Simulationsprofils;
rosbridge bindet dabei weiterhin an 127.0.0.1. Fast-DDS-UDP vermeidet geteilten Speicher
zwischen Host/Container. rosbridge transportiert Desktop-Nachrichten, DDS verbindet ROS-Nodes.
Details: [ISAAC_SIM_ROS_SETUP.md](ISAAC_SIM_ROS_SETUP.md).

## Dockerzugriff

Bei `permission_denied` zeigt der Setup-Assistent Ursache, reguläre Schritte und die
weitreichenden Rechte der Dockergruppe. Es werden keine Socketrechte automatisch geändert.
Der Benutzer kann nach bewusster Freigabe `sudo usermod -aG docker <benutzer>` ausführen,
sich vollständig neu anmelden und Studio neu starten. Rootless Docker ist eine Alternative.
Nie `chmod 666` verwenden. Ein Agent mit NoNewPrivs kann PolicyKit/sudo nicht selbst nutzen.

Unterschiede: CLI fehlt, Daemon unerreichbar, fehlende Berechtigung, aktuelles Image fehlt,
Containerstart fehlgeschlagen, Container beendet, ROS-Port nicht erreichbar oder Controller
fehlt. Status, Containerlogs und Diagnosen bleiben getrennt sichtbar.

[Docker-Postinstallation und Gruppenrechte](https://docs.docker.com/engine/install/linux-postinstall/).
Für diese Entwicklung nahm der Benutzer die Gruppenänderung und eine temporäre Benutzer-ACL
selbst vor. Nach Entfernen der ACL benötigt eine alte Sitzung weiterhin eine neue Anmeldung.
