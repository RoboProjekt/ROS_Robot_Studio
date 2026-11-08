# Multi-Domain- und Isaac-Integrationsbericht

Stand: 8. November 2026. Fortsetzung von WIP `0e93253`; keine neu aufgebaute Anwendung, kein History-Rewrite und kein Remote-Push. Die vorhandene Universal Kinematics Engine, Taskengine, ROS-Runtime und ein gemeinsamer RobotStateManager wurden weiterverwendet.

## Finale Abnahme

**185 bestanden · 0 fehlgeschlagen · 0 übersprungen**, nach der letzten Codeänderung, einschließlich echter Isaac-Simulation und der Ausfalltests aller drei ROS-Distributionen. Laufzeit: 601,88 Sekunden. Zusätzlich: **12/12 DDS-Stressrunden ohne unerwarteten Neustart**, 20 native Fensterzyklen und acht native ASan-Zyklen. Ruff Lint, Formatprüfung aller 114 Python-Dateien, Dependency Check und Git-Diff-Prüfung bestanden. [Gesamtnachweis](backend-final-verification.json).

## Tatsächlicher Funktionsumfang

Portable Backend-Profile, Domainvalidierung, getrennte Adaptercontainer/Transporte/Ports, getrennte Robot-State-Kontexte, aktive Befehlszuordnung, Quellenwechsel, GUI-Verwaltung, Diagnose und strikte Real-Robot-Lesesperren sind implementiert. Die GUI erlaubt Hinzufügen, Bearbeiten, Duplizieren, Löschen, Import/Export, Connect, Disconnect, Aktivieren, lokale Isaac-Pfadauswahl sowie Szenen- und Prozessverwaltung. MuJoCo/Gazebo sind vorbereitete Plugins und zeigen ausdrücklich **Not Implemented**.

| Prüfung | Ergebnis und Nachweis |
|---|---|
| Zwei echte SO-101-Sitzungen | Domains 10/11, verschiedene Ports/Container/Steuerkanäle, vollständiger 13-Schritt-Task, Abbruch und Quellenwechsel: [Messdaten](backend-session-verification.json) |
| Inaktive Quelle | Während des Tasks unveränderte Gelenke der zweiten Sitzung; alte Leases nach Wechsel zurückgewiesen |
| Gemeinsames Hostnetz | Domains 0/1/2 mit identischen Topicnamen, getrennte Graphen und Zustände: [Messdaten](backend-domain-verification.json) |
| Hardware-Lesesperre | Domain-0-Testquelle ohne Hardware; Goal und direkter Publish abgewiesen, drei Command-Spies jeweils null |
| Offline/Portabilität | Start/GUI ohne Simulator, lokale Pfade ausschließlich in Settings, bestehende Offline- und ROS-Demos erhalten |
| ROS-Versionen | Humble, Jazzy und Lyrical mit tatsächlichem Graph, JointStates und nativen Actions: [Humble](ros-humble-verification.json), [Jazzy](ros-jazzy-verification.json), [Lyrical](ros-lyrical-verification.json) |

Die DDS-Prüfung verwendet dasselbe Hostnetz für alle Teilnehmer. Damit wird Domain-Isolation tatsächlich geprüft; die getrennten Docker-Bridges des Zwei-Backend-Tests allein wären dafür kein ausreichender Nachweis. Das explizite UDP-Loopback-Profil vermeidet Humbles zusätzliche SHM-Transporte bei `ROS_LOCALHOST_ONLY=1`; es benötigt kein Host-IPC und keine privilegierten Container.

## ROS-Bridge-Stabilisierung

Der sporadische Humble-SIGSEGV wurde untersucht und bleibt in seiner nativen Ursache nicht eindeutig geklärt. Begrenzt überwachte Bridge-/Discovery-Kindprozesse, sichtbare Bootstrapfehler, reale Ausfalltests während einer Bewegung und ein separater Stresstest ergänzen die Fehlerbehandlung. Eine neue Verbindung setzt keine unterbrochene Bewegung automatisch fort. [Untersuchung, Grenzen und Nachweise](ROS_BRIDGE_STABILITY.md).

## Tatsächliche Isaac-Abnahme

Verwendet wurden die vorhandene **Isaac Sim 5.1.0-rc.19+release.26219.9c81211b.gl**, eine RTX 4070 Ti, **ROS 2 Humble**, Fast DDS und **Domain 1**. Isaac läuft außerhalb des Studio-Python-Prozesses. Das Original-SO-101-URDF mit 13 Original-STLs wurde durch den offiziellen Importer geladen. Sechs Gelenknamen, Typen, Achsen, Limits und Linkframes wurden gegen die existierende Kinematik geprüft. Neutraler maximaler Linkfehler: etwa **7,55·10⁻⁸ m**. Quellen wurden vor/nach Import gehasht und blieben unverändert.

Die Szene enthält Boden, Montagefläche, Licht, Kameraperspektive, feste PhysX-Articulation und den vollständigen ROS-Graph. Die Kraftantriebe verwenden Steifigkeit 5000, Dämpfung 50 und die ursprünglichen URDF-Kraftlimits von 10 Nm; PhysX verwendet 64 Positions- und 8 Geschwindigkeitsiterationen. Der zusätzliche [Existing-USD-Test](isaac-existing-scene-verification.json) bestätigt das Wiederöffnen mit unveränderter Quelldatei sowie die tatsächlichen USD-Force-Drives und Kraftlimits.

Der vollständige **13-Schritt-Pick-and-Place-Bewegungstask wurde tatsächlich erfolgreich ausgeführt**. Elf Bewegungsziele und zwei Warteschritte wurden durch die vorhandene Taskengine abgearbeitet. Arm- und Greiferbewegungen wurden aus JointStates beobachtet; Viewer und Monitoring folgten derselben Quelle. Der [aktuelle End-to-End-Bericht](isaac-backend-verification.json) enthält Rückmeldungsanzahl, akzeptierte Ziele, Ergebnisse, einzelne Konvergenzmessungen und die bestätigte Ressourcenbereinigung. Ein separater nativer Lauf erstellte [Simulatorbild](isaac-so101.png), [Verbindungsansicht](isaac-backend-connection.png) und [Monitoringbild](isaac-live-monitoring.png).

Die Nachweise werden getrennt geprüft:

1. **Goal Accepted:** Action-Server hat das konkrete Ziel angenommen.
2. **Commanded Joint State:** zeitparametrisierte SI-Positionen/Geschwindigkeiten wurden gesendet.
3. **Actual Joint State:** der Simulator liefert tatsächlich gemessene Gelenkpositionen zurück.
4. **Stable Target Reached:** mindestens fünf unterschiedliche neue Rückmeldungen liegen über mindestens 0,2 Sekunden innerhalb der unveränderten Toleranz 0,002 rad bzw. m.
5. **Result Succeeded:** der Server meldet erst danach Erfolg. Studio prüft zusätzlich seinen eigenen aktuellen State vor dem nächsten Schritt.

Am Ende wird immer die exakte Zielposition mit expliziter Geschwindigkeit null gesendet. Ein alter Sample kann nicht mehrfach zur Konvergenz beitragen; eine Toleranzverletzung setzt die Stabilitätsprüfung zurück. Der größte Gelenkfehler bei Zielbestätigung lag in den erfolgreichen Läufen bei **0,0003 rad**. Die konkreten Werte jedes Ziels stehen im JSON-Bericht. Fehlende Rückmeldungen, Pause, Clockreset und Zeitlimits bleiben Abbruchgründe. Keine Toleranzen wurden für den Test aufgeweicht.

ROS wurde während laufender Simulation getrennt und erneut verbunden. Der Simulator musste dafür nicht neu gestartet werden; alte Verbindungsleases wurden abgewiesen. Nach dem Task wurden eigene ROS-Container entfernt und der eigene Isaac-Prozess samt Kindprozessgruppe beendet. Fremde Instanzen wurden nicht beendet.

**Grenze:** Dies ist ein erfolgreicher physikalischer Gelenk-/ROS-Bewegungstask. Ein belastbarer Objektgriff mit Kontakt-, Reibungs- und Kraftvalidierung wurde nicht implementiert oder nachgewiesen. Der virtuelle Objektzustand im Studio ist kein solcher Nachweis. Isaac mit Jazzy ist vorbereitet, aber nicht als tatsächlicher Simulatorlauf geprüft; Lyrical ist für Isaac gesperrt.

## GUI- und Speicherprüfung

Die native Zwei-Backend-GUI und der native Isaac-Lauf bestanden mit echten Screenshots. Zusätzlich bestanden 20 native Fensterzyklen unter gdb ([Messdaten](gui-lifecycle-gdb-verification.json)), acht native Zyklen mit AddressSanitizer sowie abschließend 20 native Zyklen ohne Debugger: [finale native Messdaten](gui-lifecycle-verification.json), [ASan-Messdaten](gui-lifecycle-asan-verification.json). Die Gesamtsuite prüft weitere 12 generische und 20 SO-101-Zyklen mit Offscreen-Qt. Timer, verspätete ROS-Callbacks, Worker, PlotWidgets, Shaderkontexte und Widgetfreigabe werden dabei tatsächlich geprüft.

Ein früher paralleler nativer Lauf zeigte einen glibc-Heapfehler. Beim Trennen werden seitdem die Callbackreferenzen freigegeben und für ausschließlich UI-eigene Legacy-Transporte wieder sämtliche Signalverbindungen entfernt. Die genaue ursprüngliche C++-Schreiboperation konnte nicht eindeutig lokalisiert werden. Die genannten abschließenden Wiederholungen bestanden; eine allgemeine Fehlerfreiheit aller Qt-/Treiberkombinationen wird daraus nicht abgeleitet. Ein eigener AddressSanitizer-Lauf meldete keine erkannten Speicherfehler. Es wurden weder GC noch Tests deaktiviert oder Crashsignale abgefangen.

Die lokale Isaac-Installation zeigte bei aktiviertem Sampled Lighting einen Shadercache-Dekompressionsfehler mit nachfolgendem Vulkan-Speicherfehler. Die generierte einfache Szene verwendet deshalb ausdrücklich den normalen Beleuchtungspfad ohne Sampled Lighting. Dieser Pfad wurde tatsächlich gerendert; die vorhandene Installation und der Treiber wurden dafür nicht verändert. Der Importer meldet außerdem einen leeren Visual-Referenzpfad am geometriefreien `gripper_frame_link`; Gelenk-/Frameprüfung und Rendering bestehen trotzdem.

## Reproduktion und Abschluss

```bash
./start.sh --pick-place-demo
env -u PYTHONPATH .venv/bin/pytest -q --run-docker --run-model-download --run-isaac
env -u PYTHONPATH .venv/bin/ruff check src tests scripts
env -u PYTHONPATH .venv/bin/ruff format --check src tests scripts
env -u PYTHONPATH .venv/bin/python -m pip check
git diff --check
```

Ohne ausdrückliche Testoptionen werden Docker-/Download-/Isaac-Prüfungen sichtbar übersprungen; es werden keine Simulatoren automatisch installiert. Der endgültige Gesamtlauf und die Bereinigung werden in `backend-final-verification.json` festgehalten. Große USD-Assets, Images und Caches bleiben außerhalb von Git. Ein zweiter physischer Ubuntu-PC wurde nicht verwendet; Portabilität wurde über isolierte Settings, portable Projektdateien und fehlende Simulatorinstallation geprüft.

Lokale Implementierungscommits: `55bf1b0` (Sitzungen), `979973f` (GUI), `2c7b759` (DDS-Loopback), `0aa698d` (Isaac und stabile Ziele), `41df07a` (vorhandene USD-Szenen). `2737238` enthält die Transport-Lebensdauerbereinigung, `1d477f0` den Schutz ungespeicherter Projekte, `b695dd8` die begrenzte ROS-Prozessüberwachung und `a2dd24d` die Domain-Stress-/Bewegungsausfalltests. Die folgenden Abschlusscommits enthalten die Berichte und geprüften Messdaten.
