# Umsetzungsstand und Roadmap

## Funktionsfähiger MVP

- Eigenständige Qt-Desktopanwendung mit vier Tabs, vollständigem dunklem Theme,
  optionalem hellem Theme, Statusleiste, Menüs, Dateidialogen und veränderlichen Panels.
- Lokaler Bootstrap mit reproduzierbar festgelegten Python-Paketen; Start ohne pip,
  ensurepip, Docker, Host-ROS und angeschlossene Hardware.
- Fünfstufiger Installationsassistent mit Systemchecks, bewusstem Offline-Einstieg,
  überprüfbarer optionaler Docker-Installationsroutine und Runtime-Imageverwaltung.
- YAML-Projektformat v2 mit v1-Laden, atomisches Speichern, erneutes Laden, portable URDF-Kopie,
  Sensoren, TF-Offsets, Monitoring- und Runtime-Einstellungen.
- Sichere URDF-Strukturprüfung, Link-/Joint-Baum mit Limits und interaktive räumliche
  Vorwärtskinematik. Generischer Beispielroboter und vollständig nutzbarer SO-101-Offline-Viewer.
- Zentraler Robot Model Manager mit unveränderlicher Quellrevision, SHA-256-Dateiliste,
  selektiver Reparatur, atomarer Aktivierung und XDG-Modellablage.
- Originale SO-101-STLs in OpenGL, sechs virtuelle Gelenkregler, XYZ/RPY-Endeffektorpose
  und dauerhaft gespeicherte Modell-/Gelenkkonfigurationen. Keine Roboterhardwareansteuerung.
- Offline-Demo mit Gelenkzuständen, Sinussignal, Temperatur und mehreren Live-Kurven.
- rosbridge-WebSocket-Transport mit Graph-Abfragen, Typen, Nodes, Pub/Sub-Details,
  Abonnements, Reconnect, Throttling, begrenzten Puffern und kontrolliertem Freigeben.
- Implementierter Docker-SDK-Backendpfad für Humble/Jazzy/Lyrical: live geprüfte
  amd64-Manifeste, digestgebundener Build, Progress, Cache, eigene Container, Start,
  Stop, Restart, Logs, Healthchecks, begrenzte Ressourcen und loopbackgebundener Port.
- Echte rclpy-Demo-Publisher und Subscriber im Container; Ende-zu-Ende-Prüfaktion
  im Assistenten und Hauptfenster. Diese ROS-Läufe sind in dieser Umgebung ungetestet.
- Single-file-Xacro-Verarbeitung im laufenden eigenen ROS-Container; ungetestet ohne Docker.
- Deklarative Plugins mit Validierung und sichtbarem Kompatibilitäts-/Planungsstatus;
  Kompatibilitätsbericht und Schnittstelle für künftige regelbasierte Generierung.
- Unit-/GUI-/WebSocket-Tests, opt-in Docker-Integrationstests und Screenshots.

## Nächste Arbeiten

1. Auf einem Docker-fähigen Ubuntu-Rechner Humble bauen und Ende-zu-Ende-Tests ausführen;
   danach Jazzy und Lyrical. Paketquellen, QoS, Starts und Stop-Verhalten real prüfen.
2. Image-Builds in CI prüfen, Adapter-/Aptpakete über signierte Snapshots stärker pinnen,
   Paketverfügbarkeit und Gesamtdownloadgrößen zuverlässiger vorab ermitteln.
3. Abbruch/Fortsetzen laufender Builds, Registry-Authentifizierung, Recovery und manuelles
   Adoptieren eigener Restcontainer, Verwaltung mehrerer paralleler Sitzungen.
4. Plugin-Abhängigkeitsgraph und isolierter Plugin-Prozesshost; erweiterbarer
   ROS-Katalog jenseits der drei fest validierten Standarddistributionen.
5. SO-101-Hardwaretreiber, LeRobot-Anbindung, USB-Auswahl, Kalibrierung und
   sichere Treiberlebenszyklen. Bisher werden keine Hardwarebewegungen unterstützt.
6. Controller-, ros2_control-, MoveIt- und KDL-Ausführung; echte erzeugte Launch-Dateien
   und getestete ROS-Migrationsregeln mit separaten Ausgabeordnern.
7. Weitere Meshformate, Kollisions-/inertiale Darstellung, TF-Broadcasting, RViz-nahe Ansichten,
   Kamerastreams, Kalibrierung und ROS-Bag-Aufzeichnung.
8. YOLO, Tracking, Modellklassenauswahl und optionale KI-/Vision-Anbindungen.
9. Isaac-Sim-/USD-Anbindung und geprüfte Sim-to-Real-Joint-Mappings einschließlich
   Greiferkonvention; Kameraintegration und spätere kalibrierte Sensor-/Framezuordnung.
10. Native Installer/AppImage, vollständige deutsche Lokalisierung und Plattformadapter
   für Windows/WSL2 und macOS. Derzeit ist nur Ubuntu 22.04 amd64 Zielplattform.

## Bekannte Grenzen und Risiken

- Kein nutzbarer Docker-Daemonzugriff in der Entwicklungsumgebung (zuletzt fehlende
  Zugriffsrechte): keine tatsächlichen ROS-Image-Builds oder ROS-Containerläufe nachgewiesen. Registry-Verfügbarkeit allein beweist sie nicht.
- HTTPS-Zertifikatsfehler beim zusätzlichen ROS-Binärpaketindex-Probeaufruf; keine
  Zertifikatsprüfung wurde deaktiviert. Builds müssen die Paketquellen real bestätigen.
- OpenGL wird für tatsächliche Meshes benötigt; ohne Kontext wird nur die explizite
  Strukturansicht angezeigt. Floating-/Planar-Gelenke bleiben bei Nullpose. Benutzerimport
  unterstützt STL und einfache Primitive. Relative STL-Dateien werden beim Export kopiert,
  Katalogmodelle bleiben als feste model://-Referenzen im XDG-Datenverzeichnis.
- Der SO-101-Offline-Modellpfad wurde mit Originaldateien und nativer 3D-GUI verifiziert.
  Hardware-/ROS-/Isaac-Backends sind weiterhin unkonfiguriert. Weitere Katalogmodelle und
  eine mehrstufige Versionsauswahl benötigen zusätzliche überprüfte Manifeste.
- Xacro-Includes vom Host, Paketauflösung und beliebige Xacro-Workspaces werden nicht
  automatisch übertragen. Lange Xacro-Verarbeitung besitzt noch keinen GUI-Abbruch.
- Sensorparameter und Offsets werden gespeichert. TF-Publishing, dynamische TF-Quelle,
  automatische Kalibrierung und aktive Sicherheit/Schwellwertaktionen fehlen.
- Kompatibilitätsberichte prüfen deklarierte Metadaten, keine beliebige Binär-/API-Kompatibilität.
  Geplante Komponenten sind weder installiert noch ausführbar; sie bleiben als Planung sichtbar.
- Runtime-Entfernung kennt nur den aktuellen und von Studio gespeicherte/geöffnete
  Projekte. Externe unbekannte Projekte erfordern Benutzerprüfung. Es wird nie Force verwendet.
- Ein Prozessabsturz kann eigene Restcontainer hinterlassen. Normales Beenden entfernt
  nur den während dieser Sitzung gestarteten Container; Projekte liegen auf dem Host.
- Python-Abhängigkeiten sind festgelegt; Apt-Abhängigkeiten und Docker Engine selbst
  sind noch nicht vollständig reproduzierbar eingefroren.
- Der lokale rosbridge-Endpunkt hat keine zusätzliche Authentifizierung. Er wird nur
  an Loopback veröffentlicht; Remote-/Hardwarebetrieb benötigt ein separates Zugriffskonzept.
- Grenzwerte sind Beobachtungskonfigurationen und **keine zertifizierten Sicherheitsfunktionen**.
