# ROS Robot Studio

Eine startbare Desktop-Werkbank für ROS-2-Roboterprojekte. Der MVP bietet vier Tabs,
URDF-Struktur- und Meshansicht, SO-101-Follower-Modellinstallation, virtuelle Gelenksteuerung,
universelle URDF-basierte FK/IK, Teach-in, Pick-and-Place-Wizard, Offline-Trajektorien,
Sensor- und TF-Konfiguration, portable YAML-Projekte und Live-Plots.
Er funktioniert ohne Docker, ROS-Installation oder Roboterhardware.

![Pick-and-Place mit originalem SO-101 und virtuellem Objekt](docs/pick-place-editor.png)
![Live Monitoring](docs/studio-monitoring.png)

## Start unter Ubuntu 22.04

Im Projektverzeichnis:

```bash
./start.sh
```

Beim ersten Start erscheint der grafische Einrichtungsassistent. Für die sofortige
Offline-Demo ohne Assistenten:

```bash
./start.sh --demo
```

Python 3.10+, Git und eine grafische Desktop-Sitzung werden vorausgesetzt. Der Bootstrap
installiert die festgelegten Python-Abhängigkeiten ausschließlich in `.venv`; Internet ist
beim ersten Start erforderlich. Er funktioniert auch ohne pip und `python3-venv`/ensurepip.
Auf Ubuntu 22.04 amd64 wird eine gegebenenfalls fehlende kleine Qt-Cursorbibliothek aus
dem offiziellen Ubuntu-Archiv mit fester SHA-256-Prüfung lokal entpackt. Keine Systempakete,
Docker-Daemons oder ROS-Installationen werden beim Start automatisch verändert.

Die virtuelle Umgebung ist hier bereits vorbereitet. Auf einem frischen Rechner benötigt
sie ungefähr 800 MB. Docker und große ROS-Images werden nur nach ausdrücklicher Auswahl
im Assistenten beziehungsweise Runtime-Tab installiert.

## Erste Schritte

1. **Robot Configuration:** Projektname ändern, Beispiel laden oder URDF importieren.
   Links, Gelenke, Typen, Limits und Parent-/Child-Beziehungen im Baum untersuchen.
   In der Strukturansicht ziehen zum Drehen, Mausrad zum Zoomen, Doppelklick zum Zurücksetzen.
2. **Sensor Configuration:** Sensor hinzufügen oder doppelklicken. Topic, Einheit,
   Parent-Frame, Translation, RPY, statischen/dynamischen TF-Typ und Grenzwerte eingeben.
3. **Live Monitoring:** `/studio/sine`, `/joint_states` oder `/studio/temperature` auswählen.
   Numerische Felder ankreuzen, Zeitfenster einstellen, pausieren und zurücksetzen.
   Offline-Daten sind deutlich als simuliert gekennzeichnet.
4. **Speichern:** Über die Werkzeugleiste eine `.yaml`-Datei auswählen. Die URDF wird in
   einen benachbarten `.assets`-Ordner kopiert. Datei und Ordner zusammen weitergeben.
   Mit **Open…** erneut laden; ein vollständiges Beispiel liegt unter
   `examples/generic_robot/demo.studio.yaml`.
5. **Project & Runtime:** Humble auswählen, Studio-Runtime installieren, **Start ROS demo**
   drücken und **Verify ROS demo** ausführen. Danach zeigt Monitoring echte ROS-Nachrichten
   aus Demo-Nodes im Container. Mit **Stop** wird nur der eigene Container entfernt.

Das ROS-Dropdown steuert die aktuelle Sitzung. Es überschreibt den gespeicherten
Projektstandard nicht. **Save selected ROS as project default** übernimmt die Auswahl
bewusst in das Projekt. Bei laufender Containerdemo wird die alte Runtime vor einem
Versionswechsel gestoppt. Das Projekt bleibt erhalten.

## SO-101 Follower

Unter **Robot Configuration → Select robot** den SO-101 Follower auswählen. Falls nötig,
**Install Robot Model** bestätigen. URDF, 13 Original-STLs und Quellhinweise werden aus
einer festen offiziellen GitHub-Revision geprüft installiert (16,16 MB). Danach lässt
sich der Roboter vollständig offline mit sechs Reglern und numerischen Gradwerten bewegen.
XYZ/RPY des auswählbaren Endeffektorframes werden live aktualisiert. Die Original-
Geometrie benötigt OpenGL; ohne Kontext wird die Strukturansicht eindeutig angezeigt.

```bash
./start.sh --demo --robot so101_follower
```

Gelenkwerte, Modellrevision und Endeffektor werden normal mit dem Projekt gespeichert.
Das Beispiel `examples/so101/so101.studio.yaml` enthält nur eine portable Modellreferenz.
Anleitung, Lizenz, Revision, Meshdetails und Abnahmetest: [SO101.md](docs/SO101.md).

## Kinematics, Teach-in und Pick-and-Place

```bash
./start.sh --pick-place-demo
```

Öffnet die neu validierte **SO101_PickAndPlace_Demo**, bei fehlendem Modell mit
Installationsangebot und generischer Offline-Alternative. Rechts im Robot-Tab:
**Kinematics & Motion** für Chain-Analyse und kartesische Ziele, **Positions & Tasks**
für Teach-in, Schritt-Editor, den siebenstufigen Pick-and-Place-Wizard und Wiedergabe.
Die optionale SciPy-Komponente ist hier installiert. Auf einem neuen Rechner unter
**Setup assistant → Component verification → Install Universal Kinematics Support**
installieren oder `.venv/bin/python -m pip install -r requirements-kinematics.lock.txt` verwenden.

Position Only, Toolachsenrichtung und Full Pose (bei geeignetem Kettenrang) verwenden
dieselbe universelle numerische IK. Beim SO-101 bleiben fünf Armachsen vom Greifer
getrennt; Full Pose ist deaktiviert. Joint- und kartesische Positionen sowie Tasks werden
im Projektformat v3 gespeichert; v1/v2 bleiben ladbar. Alle Bewegungen sind kubische
Gelenkraumtrajektorien; der Würfel folgt einer vereinfachten visuellen Greiferkopplung.

**Collision checking not available – preview only.** Es gibt keine Hardwareausführung,
Kollisionsprüfung, Kontaktphysik oder geradlinige kartesische Bahn. ROS-/Isaac-Ausführung
sind vorbereitete Schnittstellen. [Bedienung und Grenzen](docs/KINEMATICS.md),
[aktueller Abschlussbericht](docs/KINEMATICS_REPORT.md).

## Stand und Grenzen

Der native OpenGL-Abnahmetest hat den echten SO-101 angezeigt, Qt-Regler und Kamera
betätigt, FK mit Meshtransformationen verglichen und die Pose nach Speichern/Laden
wiederhergestellt. **73 Tests bestanden (SO-101-Erweiterung vor Kinematik)**, drei ROS-Container-Tests wurden wegen fehlender
Docker-Zugriffsrechte übersprungen. [Aktueller SO-101-Bericht](docs/SO101_REPORT.md).
Der [Prüfbericht der Grundanwendung](docs/VERIFICATION.md) bleibt als historischer Stand erhalten.

Humble, Jazzy und Lyrical besitzen implementierte Runtime-Buildpfade und rosbridge-
Demoanbindung; tatsächliche ROS-Containerläufe bleiben hier ungeprüft. Das SO-101-Modell
benötigt diese Runtimes nicht. Hardware-/LeRobot-Ansteuerung, kalibrierte Joint-Mappings,
Controller-/Hardwareausführung, Isaac Sim, TF-Publishing und Kamerakalibrierung sind noch offen.
Sensorgrenzen sind **keine zertifizierten Sicherheitsfunktionen**.

## Dokumentation

- [Installation und Fehlerbehebung](INSTALLATION.md)
- [Architektur und Schnittstellen](ARCHITECTURE.md)
- [Entwicklung, Tests und Docker-Builds](DEVELOPMENT.md)
- [Projektformat](docs/PROJECT_FORMAT.md)
- [Umsetzungsstand und nächste Schritte](ROADMAP.md)
- [Kinematik und Pick-and-Place](docs/KINEMATICS.md)
- [Aktueller Kinematik-Abschlussbericht](docs/KINEMATICS_REPORT.md)
- [SO-101-Abschlussbericht](docs/SO101_REPORT.md)
- [Historischer MVP-Abschlussbericht](docs/FINAL_REPORT.md)
- [Drittanbieter-Lizenzen](THIRD_PARTY_LICENSES.md)

Studio-Quellcode und der generische Beispielroboter stehen unter Apache-2.0. Qt und
andere Abhängigkeiten behalten ihre jeweiligen Open-Source-Lizenzen.
