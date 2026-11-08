# Gemeinsamer Robot State Manager

`core.robot_state.RobotStateManager` ist die einzige aktive Zustandsquelle. Der Manager
kennt weder Qt noch ROS. `RobotState` ist ein unveränderlicher Snapshot mit schreibgeschützten
Wertzuordnungen. Er enthält Modellinstanz/Signatur/Revision/URDF, Jointnamen, Positionen,
optionale Geschwindigkeiten/Beschleunigungen/Efforts, FK-Pose und Frames, Empfangs- und
Quellzeit, Backend, Quelle, Ausführungsstatus, Fortschritt, Validität und Diagnose.

Interne Einheiten sind rad, rad/s, rad/s² bzw. m, m/s, m/s². Ein empfangener Effort wird
je nach Gelenktyp als Nm oder N angezeigt; fehlende Efforts bleiben `None`/N/A.
Keine Quelle erfindet Messwerte. Manuelles Positionieren liefert keine Geschwindigkeit.

## Aktualisierungen

- `bind(model, robot, positions, base, end)` erzeugt eine neue Modellinstanz.
- `activate_source(backend, source)` erzeugt eine neue Quellenepoche und gibt eine Lease zurück.
- `update(lease, positions, velocities=..., accelerations=...)` prüft Modell und Werte.
- `joint_state(lease, message, mapping)` bildet ROS-Namen auf URDF-Namen ab.
- `set_frames` berechnet die Darstellungspose neu, ohne Feedback künstlich aufzufrischen.
- `execution` aktualisiert Status/Fortschritt, ebenfalls ohne neue Messzeit zu behaupten.
- `invalidate`, `stale` und `snapshot` liefern Diagnose und Zustandszugriff.

Alte Leases werden nach Modell-/Backendwechsel verworfen. Updates sind durch ein RLock
serialisiert; Sequenznummern bestimmen die Zustellreihenfolge. Unbekannte/doppelte Namen,
NaN/Inf, falsche Arraylängen, Limits und widersprüchliche Mimic-Werte werden geprüft.
Fehlende unabhängige Gelenke führen zu einem ungültigen Snapshot. Die Viewer halten dann
ihre letzte gültige Pose; Tabelle und Diagnose melden fehlende/veraltete Daten.

ROS-Stamps müssen monoton steigen. Ältere oder gleiche Stamps überschreiben nichts.
Ein leerer/Null-Stamp verwendet die klar gekennzeichnete monotone Empfangszeit. Ein
Simulationszeit-Reset verlangt eine neue explizite Quellenepoche/Verbindung. ROS-Zustände
altern standardmäßig nach einer Sekunde; lokale statische Posen bleiben gültig.

## GUI-Verteilung

`gui.robot_monitor.StateSignals` übergibt Snapshots mittels Qt-Signal an den GUI-Thread.
MainWindow aktualisiert Gelenkregler, Robot Configuration, primären Viewer und Monitoring.
Die kompakte Monitoring-Ansicht verwendet dieselbe `RobotView`-Klasse und dasselbe Modell;
sie hält nur den zur Darstellung benötigten Gelenkwertcache. Kein Viewer ist die Zustandsautorität.

Offline-Playback veröffentlicht Position und **analytische** Ableitungen des bestehenden
Cubic-Segments. Pause/Stop halten die Pose mit berechneter Ruhegeschwindigkeit null.
Eine reine IK-Berechnung verändert den Manager nicht. Offline-Zielvorschau ist ausdrücklich
als Ziel markiert; erst Apply übernimmt den Zustand. Im ROS-Modus sind lokale Apply-/Preview-
Übernahmen gesperrt: beide Viewer folgen ausschließlich `/joint_states`.

ROS-Callback/Transportinstanz und Lease werden beim Quellenwechsel getrennt. Der alte
Offline-Player ist während ROS-Ausführung inaktiv. Eine automatische Wiederverbindung
setzt keinen unterbrochenen Task fort. Modellwechsel bricht die alte Ausführung ab.

Prüfungen: `tests/test_robot_state.py`, `tests/test_motion_gui.py`, reale ROS-Abnahme über
`scripts/verify_ros_runtime.py`. Architekturüberblick: [ARCHITECTURE.md](../ARCHITECTURE.md).
