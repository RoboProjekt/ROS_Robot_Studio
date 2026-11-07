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
