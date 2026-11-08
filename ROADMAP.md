# Umsetzungsstand und Roadmap

## Funktionsfähiger MVP

- Eigenständige Qt-Desktopanwendung mit vier Tabs, vollständigem dunklem Theme,
  optionalem hellem Theme, Statusleiste, Menüs, Dateidialogen und veränderlichen Panels.
- Lokaler Bootstrap mit reproduzierbar festgelegten Python-Paketen; Start ohne pip,
  ensurepip, Docker, Host-ROS und angeschlossene Hardware.
- Fünfstufiger Installationsassistent mit Systemchecks, bewusstem Offline-Einstieg,
  überprüfbarer optionaler Docker-Installationsroutine und Runtime-Imageverwaltung.
- YAML-Projektformat v3 mit v1-/v2-Migration, atomisches Speichern, erneutes Laden, portable URDF-Kopie,
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
  im Assistenten und Hauptfenster. Humble, Jazzy und Lyrical sind inzwischen real geprüft.
- Single-file-Xacro-Verarbeitung im laufenden eigenen ROS-Container; ungetestet ohne Docker.
- Deklarative Plugins mit Validierung und sichtbarem Kompatibilitäts-/Planungsstatus;
  Kompatibilitätsbericht und Schnittstelle für künftige regelbasierte Generierung.
- Unit-/GUI-/WebSocket-Tests, opt-in Docker-Integrationstests und Screenshots.

## Implementierte Offline-Bewegungsprogrammierung

- Universelle URDF-Kettenanalyse mit Arm/Greifer-Trennung und numerischen Fähigkeits-/Singularitätstests.
- Optionaler SciPy-1.15.3-Solver: Position, Toolachsenrichtung, Full Pose bei geeignetem Rang,
  Bounds, reproduzierbare Mehrfachstarts und unabhängige FK-Restfehlerprüfung.
- Kartesische GUI im vorhandenen Robot-Tab, Teach-in-Manager, siebenstufiger Pick-and-Place-Wizard.
- Sequentielle Tasks mit sieben Aktionstypen, Einzel-/Gesamtvalidierung, Bibliothek und Projektpersistenz.
- Synchronisierte kubische Gelenkraumtrajektorien mit wirklicher Velocity-/Accelerationprüfung,
  Pause/Resume/Stop/Reset und vereinfachtem Greifobjekt.
- Geprüfte SO101_PickAndPlace_Demo, generische 3P-Demo und Full-Pose-Test eines eigenen 6R-Arms.
- Native GUI-Abnahme und Screenshots; [aktueller Bericht](docs/KINEMATICS_REPORT.md).

## Implementierte ROS-Runtime und Synchronisierung

- Gemeinsamer thread-sicherer Robot State Manager mit Modell-/Quellen-Leases und FK.
- Robot-State-Monitoring als Standard, dynamische Gelenktabelle, kompakter Originalmodell-
  Viewer, bounded Positions-/Velocity-/Effort-/Toolplots; Sinusdemo explizit ausgelagert.
- Tatsächlich gebaute Humble-/Jazzy-/Lyrical-Runtimes; native FollowJointTrajectory-Action,
  generischer ROS-Simulationscontroller und echte SO-101-Pick-and-Place-Rückmeldungen.
- Wiederholter Demo/ROS-Wechsel, Abbruch, Heartbeat-/Zeit-/Startzustandsprüfung und
  aktuelle Image-Fingerabdrücke. Verified nur nach tatsächlicher Diagnose.
- Explizite externe Simulation, DDS-/Mappingdialog und optionaler JointState-Kommandopfad.
  Dieser ist gegen einen ROS-Szenenemulator geprüft; echte Isaac-SO-101-Szene weiterhin offen.

## Nächste Arbeiten

1. Die inzwischen real geprüften Humble-/Jazzy-/Lyrical-Builds in CI wiederholen;
   QoS, Netzunterbrechungen, größere Robotermodelle und langfristige Laufzeiten erweitern.
2. Image-Builds in CI prüfen, Adapter-/Aptpakete über signierte Snapshots stärker pinnen,
   Paketverfügbarkeit und Gesamtdownloadgrößen zuverlässiger vorab ermitteln.
3. Abbruch/Fortsetzen laufender Builds, Registry-Authentifizierung, Recovery und manuelles
   Adoptieren eigener Restcontainer, Verwaltung mehrerer paralleler Sitzungen.
4. Plugin-Abhängigkeitsgraph und isolierter Plugin-Prozesshost; erweiterbarer
   ROS-Katalog jenseits der drei fest validierten Standarddistributionen.
5. SO-101-Hardwaretreiber, LeRobot-Anbindung, USB-Auswahl, Kalibrierung und
   sichere Treiberlebenszyklen. Bisher werden keine Hardwarebewegungen unterstützt.
6. Kollisionsprüfung, kartesische Zwischenpunktplanung/Gizmo und verifizierte externe 6R-Modelle;
   Controller-, ros2_control-, MoveIt- und KDL-Ausführung; echte erzeugte Launch-Dateien
   und getestete ROS-Migrationsregeln mit separaten Ausgabeordnern.
7. Weitere Meshformate, Kollisions-/inertiale Darstellung, TF-Broadcasting, RViz-nahe Ansichten,
   Kamerastreams, Kalibrierung und ROS-Bag-Aufzeichnung.
8. YOLO, Tracking, Modellklassenauswahl und optionale KI-/Vision-Anbindungen.
9. Isaac-Sim-/USD-Anbindung und geprüfte Sim-to-Real-Joint-Mappings einschließlich
   Greiferkonvention; Kameraintegration und spätere kalibrierte Sensor-/Framezuordnung.
10. Native Installer/AppImage, vollständige deutsche Lokalisierung und Plattformadapter
   für Windows/WSL2 und macOS. Derzeit ist nur Ubuntu 22.04 amd64 Zielplattform.

## Bekannte Grenzen und Risiken

- Der frühere Docker-Berechtigungsblocker wurde mit ausdrücklicher Benutzerfreigabe behoben.
  Alle drei Runtimes sind real getestet. Bestehende Sitzungen benötigen nach Gruppenänderung
  weiterhin eine neue Anmeldung; ein erfolgreicher Build allein bedeutet keine Verified-Abnahme.
- Ein früherer zusätzlicher Host-Paketindex-Probeaufruf hatte einen TLS-Fehler. Die echten
  Container-Builds haben inzwischen die signierten Paketquellen erfolgreich verwendet;
  Zertifikatsprüfungen wurden nicht abgeschaltet.
- OpenGL wird für tatsächliche Meshes benötigt; ohne Kontext wird nur die explizite
  Strukturansicht angezeigt. Floating-/Planar-Gelenke bleiben bei Nullpose. Benutzerimport
  unterstützt STL und einfache Primitive. Relative STL-Dateien werden beim Export kopiert,
  Katalogmodelle bleiben als feste model://-Referenzen im XDG-Datenverzeichnis.
- Der SO-101-Offline-Modellpfad wurde mit Originaldateien und nativer 3D-GUI verifiziert.
  Hardware- und Isaac-Szenenverbindung sind weiterhin unkonfiguriert; ROS-Simulation ist implementiert. Weitere Katalogmodelle und
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

Offline-Tasks sind kinematisch geprüft, aber weder kollisionsfrei noch hardwarebereit.
Full Pose wird für den SO-101 bewusst nicht angeboten. Numerische IK kann lokale Minima
liefern und beweist keine globale Nichterreichbarkeit. Kamera-3D-Lokalisierung, dynamische
TF-Provider, Sensortrigger, Schleifen und Verzweigungen bleiben offen. Details und
Konventionen: [KINEMATICS.md](docs/KINEMATICS.md).

ROS-Action-Pause/Resume ist noch nicht implementiert; Stop cancelt, anschließend wird neu
geplant. Kollisionsprüfung, Hardwarefreigabe und kalibrierte Sim-to-Real-Mappings bleiben
zukünftige Arbeiten. [Aktueller Bericht](docs/ROS_RUNTIME_VERIFICATION.md).
