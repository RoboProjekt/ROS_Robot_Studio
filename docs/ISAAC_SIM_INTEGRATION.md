# Isaac Sim 5.1 integrieren

Isaac bleibt ein eigener Prozess. `IsaacSimInstallationManager` prüft Version 5.1, ausführbare offizielle Launcher, URDF-Importer und Humble/Jazzy-Bridge. Er sucht konfigurierte Pfade, `ISAAC_PATH` sowie übliche Benutzerinstallationen. Die Studio-.venv importiert keine Omniverse-/Isaac-Module. Lyrical ist für diesen Adapter nicht freigegeben.

`IsaacSimProcessManager` startet das offizielle `python.sh` über einen kleinen Prozessbetreuer. Dieser besitzt eine eigene Kindprozessgruppe, leitet Stop an diese Gruppe weiter und wartet auf das Ende; nach zwölf Sekunden beendet er verbleibende eigene Kinder. Fremde Simulatorprozesse werden nicht gesucht oder beendet. Die ROS-Domain und Bridge-Bibliotheken werden vor Start gesetzt; fremde Python-/ROS-Workspace-Umgebungen werden entfernt. Qt erhält begrenzte Prozessprotokolle und einen geprüften Scene-ready-Bericht.

`IsaacSimSceneManager` erstellt aus dem aktuell zugeordneten Originalmodell einen lokalen Prüfauftrag. Die existierende URDF-Kinematik liefert erwartete Gelenkachsen und Linkpositionen; es gibt keine zusätzliche FK-Implementierung. SHA-256-Prüfungen sichern URDF und Meshquellen sowie eine ausgewählte vorhandene USD-Quelle vor und nach dem Import. Request-ID, Modellsignatur und Domain verhindern die Verwendung alter Szenenberichte.

Der offizielle URDF-Importer erzeugt USD mit fester Basis, Originalgelenken und Limits. Die Szene enthält einen prozeduralen Montagetisch, Boden, Licht und den ROS-Action-Graph: Playback-Tick, eigener ROS2Context, JointState-Publisher, JointState-Subscriber, ArticulationController und Clock-Publisher. Sie benötigt keine heruntergeladenen Umgebungsassets. Jede gleichzeitig gestartete Instanz benötigt ein eigenes Ausgabeverzeichnis; die Standardpfade enthalten die Profil-ID. Der Import wird vor dynamischer Belastung ohne Schwerkraft vermessen; danach läuft die Simulation mit normaler Schwerkraft.

Ein vorhandenes USD wird als Bühne geöffnet und in ein separates Ausgabeverzeichnis exportiert. Es muss Meter, Z-up und genau eine passende Articulation unter `/World/Robot` besitzen; Gelenknamen, Typen, Achsen, Limits und Linkframes werden erneut geprüft. `/World/ROS2` wird in der abgeleiteten Szene mit dem aktuellen Domain-/Topic-Graph eingerichtet. Die Quelldatei bleibt unverändert. Der automatische Import verwendet unveränderte URDF-Gelenknamen und verlangt Identitätsmapping; benutzerdefinierte externe Szenen können ihr ROS-Mapping selbst anbieten.

Die ROS-Runtime führt die vorhandene FollowJointTrajectory-Ausführung über `IsaacSimROSAdapter` aus: validierte Trajektorie → SI-JointState-Kommandos → tatsächliche Gelenkrückmeldung. Erfolg verlangt beobachtete Zielpositionen. Alte Rückmeldungen, fehlende Clock, Pause oder Reset führen zum Abbruch; nach Reset neu verbinden und planen.

Die lokale Prüfung verwendet Isaac **5.1.0-rc.19+release.26219**, RTX 4070 Ti und **ROS 2 Humble, Domain 1**. Gelenknamen, Achsen und Limits des originalen SO-101 stimmen; maximaler neutraler Linkfehler beträgt etwa **7,55·10⁻⁸ m**. Bewegungsnachweise und Taskstatus stehen im [Prüfbericht](BACKEND_VERIFICATION.md). Ein physikalisch belastbarer Objektgriff und Kontakt-/Kraftvalidierung sind ausdrücklich kein bereits nachgewiesenes Ergebnis.

Reproduktion mit vorhandener Installation und Runtime:

```bash
env -u PYTHONPATH QT_QPA_PLATFORM=offscreen .venv/bin/python scripts/verify_isaac_backend.py
```

Grundlage: offizielle [5.1-ROS-Installation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html), [Joint Control](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_manipulation.html) und die lokal installierten 5.1-APIs. USD und Simulatorassets liegen im lokalen Cache und werden nicht eingecheckt.

Die finalen Kraftantriebe verwenden Steifigkeit 5000 und Dämpfung 50 bei unveränderten URDF-Kraftlimits. Am Trajektorienende kommandiert der ROS-Adapter die exakte Position mit Geschwindigkeit null und verlangt mindestens fünf neue Samples über mindestens 0,2 Sekunden innerhalb der unveränderten Toleranz. Die einzelnen Zielbefunde werden im Action-Ergebnis und im Integrationsbericht festgehalten.
