# Installation und Einrichtungsassistent

## Desktop-Bootstrap

Im Repository `./start.sh` aufrufen. Das Skript erstellt lokal `.venv`, bootstrapped
bei Bedarf pip aus einem SHA-256-geprüften Wheel und installiert die festgelegten
Abhängigkeiten. Beim erneuten Start werden bestehende Pakete wiederverwendet.

Ubuntu 22.04 benötigt für Qt/X11 unter anderem `libxcb-cursor0`. Falls nur diese kleine
Bibliothek fehlt, wird das festgelegte Ubuntu-amd64-Paket nach `.venv/native` entpackt
und nur dem Anwendungsprozess über `LD_LIBRARY_PATH` bereitgestellt. Der Paketmanager
und globale Suchpfade werden nicht verändert. Weitere fehlende Desktopbibliotheken
werden nicht automatisch als Systempakete installiert; die Fehlermeldung von Qt/`ldd`
ist dann zu prüfen. Headless-Tests verwenden `QT_QPA_PLATFORM=offscreen`.

## Fünf Schritte

1. **System:** OS, Architektur, Python, Internet, Speicherplatz, Docker-CLI und
   SDK-/Daemonzugriff inklusive Berechtigungsfehler prüfen. Die Prüfungen sind nur lesend.
2. **Docker:** Bestehenden erreichbaren Daemon verwenden. Optionalen Installer vorher
   anzeigen lassen und ausdrücklich autorisieren. `pkexec` fordert Administratorrechte
   über PolicyKit an. Ohne PolicyKit kann die Routine nach Durchsicht von einem
   Administrator separat ausgeführt werden. Sie ist auf Ubuntu 22.04 amd64 begrenzt,
   nutzt Docker-Paketquellen und entfernt keine konfliktbehafteten Installationen.
3. **ROS-Runtimes:** Humble, Jazzy, Lyrical sind vorgewählt. **Check image availability &
   size** ermittelt das echte amd64-Manifest und die komprimierte Basisgröße. **Install
   selected runtimes** lädt die ausgewählten Images und baut die Erweiterungen. Einzelne
   Versionen können abgewählt und später installiert werden. Nicht erreichbare Images
   werden nach der Prüfung abgewählt; erneut prüfen erlaubt einen späteren Versuch.
4. **Komponenten:** Installationsstatus abrufen. **Verify selected ROS runtime** startet
   einen temporären eigenen Container und prüft ROS-Graph, Demo-Subscriber-Node und
   zwei echte Topics über den GUI-WebSocket-Adapter. Der Container wird danach entfernt.
   Ein Image-Build wird ohne diesen Test nicht als erfolgreicher ROS-Lauf ausgegeben.
5. **Abschluss:** Installations- und Fehlerbericht ansehen und optional Offline-Demo öffnen.
   Der Bericht wird in den XDG-Einstellungen gespeichert. **Help → Setup assistant**
   öffnet den Assistenten erneut.

Downloads können nach Fehlern wiederholt werden; Docker nutzt vorhandene Layer.
Während eines laufenden Builds sind Assistent-Schließen und Navigation gesperrt, die
Ereignisverarbeitung bleibt aktiv. Eine feinere Download-Abbruchfunktion ist noch offen.
Die komprimierte Basisgröße ist kein Gesamtbedarf: ROS-Zusatzpakete, entpackte Layer
und Buildcache benötigen zusätzlichen Speicher. Mehrere GiB pro Runtime einplanen.

## Unterstützte Runtimes

| ROS 2 | Container-Ubuntu | Offizielles Basisimage | Plattform |
| --- | --- | --- | --- |
| Humble | 22.04 Jammy | `ros:humble-ros-base-jammy` | linux/amd64 |
| Jazzy | 24.04 Noble | `ros:jazzy-ros-base-noble` | linux/amd64 |
| Lyrical | 26.04 Resolute | `ros:lyrical-ros-base-resolute` | linux/amd64 |

Quellen und Prüfstatus: [VERIFICATION.md](docs/VERIFICATION.md). Der Katalog markiert
EOL anhand des eingetragenen Supportdatums und sperrt neue Installationen. Eine freie
Erweiterung um weitere Distributionen ist für einen späteren Katalogadapter vorgesehen.
Im MVP sind die drei oben genannten Distributionen fest validiert.

Die Studio-Images installieren `ros-<distro>-rosbridge-suite` und `ros-<distro>-xacro`.
Die Paketverfügbarkeit wird im Build per Apt geprüft. Der Build bricht bei fehlenden
Paketen oder fehlerhaften Quellen sichtbar ab. Keine ROS-Python-Pakete werden auf dem
Host installiert. Der bestehende Inhalt von `/opt/ros` wird nicht benötigt oder verändert.

## Docker-Berechtigungen

Docker-Daemonzugriff ist sicherheitsrelevant. Die Anwendung fügt den Benutzer niemals
still der `docker`-Gruppe hinzu und startet die GUI nicht als root. Ein Administrator
muss einen geeigneten Zugriffsweg bereitstellen. Für eine bereits konfigurierte rootless
Engine zum Beispiel `DOCKER_HOST=unix:///run/user/$UID/docker.sock ./start.sh` verwenden.
Docker-CLI-Kontexte werden vom Python-SDK nicht automatisch ausgelesen.

Wenn ein Administrator bewusst die docker-Gruppe verwendet, ist deren weitreichender
Hostzugriff zu berücksichtigen; neue Gruppenmitgliedschaften erfordern eine neue
Anmeldung. Siehe [Docker-Postinstallation](https://docs.docker.com/engine/install/linux-postinstall/)
und [Rootless Docker](https://docs.docker.com/engine/security/rootless/).

## Fehlerbehebung

- **Docker unavailable:** Offline-Demo bleibt nutzbar. Fehlermeldung im Runtime-Log prüfen:
  fehlender Socket, gestoppter Dienst und fehlende Zugriffsrechte werden weitergegeben.
- **ROS verbindet nicht:** Studio-Image installiert? Containerlogs abrufen; rosbridge und
  rosapi prüfen. Der Client versucht kontrolliert erneut zu verbinden. Portkonflikte unter
  Settings durch einen anderen Port lösen und Runtime neu starten.
- **Registry-/Paketfehler:** Internet, DNS, Proxy und Zertifikate prüfen. TLS-Prüfung nicht
  ausschalten. Ein abgebrochener Build verändert keine Projektdateien.
- **URDF abgewiesen:** Nur zusammenhängende Bäume mit eindeutigen Links/Gelenken importieren.
  Revolute-/Prismatic-Gelenke benötigen gültige Grenzen. Externe XML-Entities sind gesperrt.
- **Xacro:** Laufender Studio-Container erforderlich. Nur einzelne Dateien werden
  übertragen; lokale Include-Bäume werden im MVP nicht automatisch kopiert.
- **Runtime deinstallieren:** Aktueller und bekannte gespeicherte/geöffnete Projekte werden
  geprüft; Referenzen verhindern die Entfernung. Unbekannte externe Projekte sind nicht
  auffindbar. Docker blockiert zusätzlich die Entfernung referenzierter Images, ohne Force.
- **Absturz/Restcontainer:** Nur explizit identifizierte eigene Studio-Container verwalten.
  Eine Recovery-GUI ist noch nicht vorhanden. Fremde Container bleiben unberührt.


## SO-101-Modell ohne ROS oder Docker installieren

Die Desktop-Abhängigkeiten enthalten zusätzlich trimesh 4.6.8, PyOpenGL 3.1.9 und
filelock 3.18.0. `start.sh` aktualisiert ausschließlich die lokale virtuelle Umgebung.
Weder LeRobot noch Isaac Sim sind erforderlich. Unter **Select robot → SO-101 Follower**
**Install Robot Model** bestätigen; der Installer lädt 16,16 MB Originaldateien von einer
festen Revision. Der Modelldownload ist vom ROS-Installationsassistenten getrennt.

Cache: `${XDG_DATA_HOME:-~/.local/share}/ros-robot-studio/robots/so101_follower/<revision>/`.
Das Modell ist auf diesem Entwicklungsrechner bereits installiert und geprüft.
Intakte Dateien werden beim Neustart nicht erneut heruntergeladen. Beschädigte oder
fehlende Dateien können über **Repair Robot Model** ersetzt werden. Bei fehlender
Installation eines gespeicherten Projekts bietet Studio die gleiche Revision erneut an.

Für tatsächliche Meshes ist ein OpenGL-Compatibility-Kontext ab 2.1 erforderlich.
Ohne Kontext bleibt die beschriftete Strukturansicht erhalten; es wird keine vollständige
Geometrie vorgetäuscht. Kamera und alle Offline-Regler benötigen keine ROS-Verbindung.
`package://`-Assets eigener URDFs benötigen eine explizite Paketwurzel im Loader-API;
eine grafische Paketzuordnung ist noch nicht vorhanden.

Details, Originalquelle, Prüfsummen und Kalibrierkonvention: [SO101.md](docs/SO101.md).

## ROS-Simulation und Dockerzugriff

Die drei Studio-Runtimes können im Setup-Assistenten oder unter Project & Runtime
installiert werden. Standard ist Humble; Jazzy und Lyrical bleiben eigenständige Images.
Auf diesem Rechner wurden alle drei Images gebaut und echte Task-/Feedbacktests ausgeführt.

Für regulären Dockerzugriff muss die Desktop-Sitzung die bewusst freigegebene Dockergruppe
besitzen. Nach `sudo usermod -aG docker local-user` vollständig ab-/anmelden und Studio neu starten.
Die Gruppe gewährt weitreichende Hostrechte. Rootless Docker ist eine Alternative.
Studio ändert Gruppen oder Socketrechte nicht automatisch. Niemals den Socket weltweit öffnen.

Ein bereits laufender Agent kann mit `NoNewPrivs` weder PolicyKit/sudo ausführen noch seine
alten Gruppen neu laden. Während dieser Entwicklung hat der Benutzer deshalb vorübergehend
`sudo setfacl -m u:local-user:rw /var/run/docker.sock` ausgeführt. Nach neuer Anmeldung kann diese
zusätzliche ACL mit `sudo setfacl -x u:local-user /var/run/docker.sock` entfernt werden. Das sind
bewusste Benutzeraktionen, keine automatische Installation oder Berechtigungsumgehung.

Nach dem Start **Open Live ROS Demo** wählen. Die vorhandenen Buttons **Offline demo**,
**Stop** und **Restart** verwalten Quellen und eigene Container wirklich. Datenquelle,
Controller und Executing/Ready bleiben getrennt sichtbar. Manuelle Regler ändern keinen
aktiven ROS-Zustand. Vor neuer Bewegung sind aktuelle JointStates und Controller nötig.

Erläuterungen: [ROS Runtime](docs/ROS_RUNTIME.md), [Ausführung](docs/ROS_EXECUTION.md),
[Abnahme und Fehlergrenzen](docs/ROS_RUNTIME_VERIFICATION.md). Isaac Sim 5.1 wurde im
Dateisystem gefunden; eine konfigurierte SO-101-Szene ist nicht nachgewiesen. Sein Setup
bleibt separat: [Isaac ROS Setup](docs/ISAAC_SIM_ROS_SETUP.md).

Die Qt-Pakete PySide6, Addons, Essentials und Shiboken müssen gemeinsam Version **6.9.3**
verwenden; `./start.sh --prepare-only` übernimmt den geprüften Lockstand. Historische
6.8.3-Installationen werden ausschließlich in `.venv` aktualisiert. Details zur
[GUI-Stabilisierung](docs/GUI_STABILITY_REPORT.md).

## Optionale externe Simulatoren

Isaac Sim ist keine Startvoraussetzung. Im Backend Connection Manager kann eine vorhandene 5.1-Installation erkannt oder manuell gewählt werden. Installationspfad und Szenenpfad werden lokal pro Profil-ID gespeichert; beim Übertragen eines Projekts die lokale Zuordnung erneut auswählen. Humble ist die erste tatsächlich verwendete Isaac-ROS-Version; Jazzy ist im Adapter vorgesehen, Lyrical für Isaac gesperrt. Keine Isaac-/Omniverse-Pakete in die Studio-.venv installieren.

Mehrere ROS-Verbindungen verwenden separate Adaptercontainer und freie Loopback-Ports. Externe DDS-Verbindungen benötigen die ausdrückliche Hostnetzfreigabe des Profils. Lokales DDS nutzt UDP-Loopback ohne Host-IPC. Dockerzugriff über die bereits eingerichtete Benutzergruppe verwenden; der normale Programmstart verändert keine Socketrechte. [Bedienung](docs/BACKEND_CONNECTION_MANAGER.md), [Netzwerkdetails](docs/ROS_MULTI_DOMAIN.md).
