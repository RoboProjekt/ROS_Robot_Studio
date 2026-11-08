# GUI-Stabilisierung und Lebensdauertests

Aktuelle Multi-Domain-/Isaac-Abnahme: [BACKEND_VERIFICATION.md](BACKEND_VERIFICATION.md). Die folgenden ursprünglichen Ergebnisse werden durch die dort dokumentierten späteren nativen, gdb- und ASan-Läufe ergänzt.

Stand: 8. November 2026. Fortsetzung des vorhandenen Entwicklungsstands; keine neue
Anwendung, kein Remote-Push, keine globalen Python-Paketänderungen.

## Befund und Korrektur

Der bisherige Offscreen-Regressionslauf stürzte sporadisch nativ ab. Mehrere gdb-Läufe
zeigten den GUI-Thread in `QCoreApplication::notifyInternal2` /
`QTimerInfoList::activateTimers`, ein weiterer in `QApplication::notify`.
Die Speicherbeschädigung war damit eingegrenzt, aber nicht auf eine einzelne
C++-Schreiboperation oder einen bestimmten Upstream-Bug zurückgeführt.
Plot-Schließen allein auszubauen verschob den Fehler und führte auch zu einem Hänger;
dieser Versuch wurde verworfen. Automatische Garbage Collection blieb eingeschaltet.

Nachgewiesene Lebensdauerfehler wurden konkret behoben:

- `MainWindow.closeEvent` ist idempotent und stoppt auch den ROS-Startup-Timer.
  Bereits zugestellte Signalereignisse können nach einem Disconnect noch in Qt warten.
  Geschlossene Fenster nehmen deshalb keine ROS-Pakete, State-Updates, Runtime-Refreshes,
  Startfehler, Healthchecks oder Playback-Ticks mehr an.
- Kurvenreferenzen werden vor dem Plotabbau freigegeben; Auswahl-Signale werden gesperrt.
  `gui.plots.close_plot` schließt den ViewBox-Linkeintrag und merkt sich vor dem
  PyQtGraph-Schließen dessen Menüs/Grafikobjekte. PyQtGraph 0.13.7 entfernt diese Objekte
  teilweise aus der Szene bzw. löst ihren Parent. Die verbliebenen gültigen QObjects und
  das abgetrennte PlotWidget werden anschließend über `deleteLater` im GUI-Thread gelöscht.
- Die verborgene frühere `follow_live`-Checkbox hat nun einen QWidget-Parent.
  Der Stresstest prüft die tatsächliche Rückkehr auf die Ausgangsanzahl von Qt-Widgets.
- Der native Test fand außerdem einen reproduzierbaren OpenGL-Fehler ab dem zweiten
  Hauptfenster: Der globale benannte PyQtGraph-Shader enthielt eine Programm-ID aus dem
  bereits zerstörten Kontext. `studio_shader` erzeugt nun ein vom jeweiligen Viewer
  gehaltenes ShaderProgram; dessen kompilierte ID wird nicht prozessweit wiederverwendet.
  Der Test prüft in jedem neuen Kontext `glIsProgram`, `glGetError` und echte Framebuffer.

## Qt-Versionen und Kompatibilität

PySide6, Addons, Essentials und Shiboken sind gemeinsam auf **6.9.3** gepinnt und in
`pyproject.toml`, `requirements.lock.txt`, Lizenzübersicht und lokaler `.venv` konsistent.
PyQtGraph bleibt **0.13.7**, NumPy **2.2.6**, PyOpenGL **3.1.9**, Python **3.10**.

Qt dokumentiert ab 6.9.2 die Korrektur PYSIDE-3148 für Speicherbeschädigungen beim
Verbinden mehrerer Signale mit einem Python-Empfänger ohne QObject-Basis:
[offizielle Release Notes](https://doc.qt.io/qtforpython-6.10/release_notes/pyside6_release_notes.html).
Das ist ein relevanter zusätzlicher Grund, 6.8.3 hier zu ersetzen; die Zuordnung des
ursprünglichen Studio-Absturzes exakt zu PYSIDE-3148 ist eine Vermutung, kein bewiesener
C++-Minimalfall. Mit 6.9.3 bestanden zunächst fünf unabhängige Wiederholungen derselben
26 GUI-/State-/Transporttests, die unter 6.8.3 mehrfach nativ abstürzten.

Es wurden weder Tests deaktiviert noch Crashsignale abgefangen, GC deaktiviert oder
Qt-Objekte absichtlich am Leben gehalten. Dependency-Checks müssen mit
`env -u PYTHONPATH` laufen, damit eine gesourcte Host-ROS-Umgebung nicht in die Desktop-
venv hineinwirkt. Die Anwendung startet ebenfalls ohne geerbtes ROS-PYTHONPATH.

## Gezielter Regressionstest

`scripts/verify_gui_lifecycle.py` erzeugt und zerstört im selben Prozess wiederholt
vollständige MainWindows mit dem vorhandenen Projekt, Monitoring, Positions-/Velocity-
Kurven, Taskwiedergabe, Tabwechsel und Pick-and-Place-Wizard samt Worker. Zum Schließen
stehen echte RosbridgeTransport-Flush-/Socket-Timer, explizit gequeue-te JointState-
und State-Signale sowie Runtime-/Startup-Callbacks an. Die Pakete werden injiziert;
dieser Test beansprucht keine echte DDS-Verbindung.

Jeder Zyklus prüft gestoppte Timer, beendete Threadpools, unveränderten State nach Close,
zerstörte Window-/Transport-/Wizard-/Plot-QObjects und keine verbleibenden Widgets.
GC wird nach kontrollierter DeferredDelete-Verarbeitung regulär ausgeführt.
Ein separater Python-Prozess macht einen nativen Absturz zum fehlgeschlagenen Pytest.

```bash
# Automatisch: 12 generische und 20 originale SO-101-Zyklen ohne OpenGL.
env -u PYTHONPATH QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest -q \
  tests/test_gui_lifecycle.py --run-model-download
# Native Prüfung mit originalen STL-Meshes und OpenGL-Kontexten:
env -u PYTHONPATH QT_QPA_PLATFORM=xcb \
  LD_LIBRARY_PATH="$PWD/.venv/native/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  .venv/bin/python scripts/verify_gui_lifecycle.py --robot so101 --cycles 20 \
  --report docs/gui-lifecycle-verification.json
```

Ergebnis der vollständigen Regression und nativen Abnahme: siehe
[Runtime-Abschlussbericht](ROS_RUNTIME_VERIFICATION.md) und
[maschinenlesbares natives Lebensdauerprotokoll](gui-lifecycle-verification.json).
Das lokale QWebSocket-Abbruch-Szenario kann die Qt-Debugmeldung
`QObject::disconnect: wildcard call disconnects from destroyed signal of QTcpSocket::unnamed`
ausgeben. Diese wird nicht unterdrückt; sie ist keine fehlgeschlagene Render-/Freigabeprüfung.

## Umfang und Grenzen

ROS-Code, native FollowJointTrajectory-Actions, Controller und gebaute Docker-Images
wurden bei der Qt-Stabilisierung erhalten. Echte ROS-Abnahmen laufen gesondert mit
Humble, Jazzy und Lyrical; sie ersetzen die injizierten Pakete durch tatsächliche DDS-
JointStates/Actions. Isaac Sim 5.1 ist lokal vorhanden, aber ohne nachgewiesene kompatible
SO-101-Szene bleibt dessen echte Integration **nicht getestet**. Der isolierte ROS-
Szenenemulator ist ausdrücklich kein Isaac-Beweis. Hardware bleibt gesperrt.

## Geänderte Dateien der Stabilisierung

- `src/ros_robot_studio/gui/main_window.py`: geschlossenes Fenster, verspätete Callbacks,
  Startup-Timer und idempotentes Schließen.
- `src/ros_robot_studio/gui/plots.py`: explizite Freigabe abgetrennter Plot-QObjects.
- `src/ros_robot_studio/gui/robot_monitor.py`, `panels.py`, `motion_workspace.py`:
  Monitoring-Shutdown, Checkbox-Ownership und Tick-Gültigkeit.
- `src/ros_robot_studio/gui/robot_view.py`: Shaderprogramm pro Viewer/OpenGL-Lebensdauer.
- `pyproject.toml`, `requirements.lock.txt`, `THIRD_PARTY_LICENSES.md`: konsistenter Qt-Pin.
- `scripts/verify_gui_lifecycle.py`, `tests/test_gui_lifecycle.py`, `tests/test_motion_gui.py`:
  wiederholte vollständige Ressourcenzerstörung und ausstehende Nachrichten.
- `scripts/verify_ros_runtime.py`: aktueller Ausschnitt der Positions-/Velocitykurven.
- README, Entwicklungs-/Installations-/Architekturdokumentation, Roadmap und `docs/`:
  Abschluss der bereits begonnenen Berichte, aktuelle Prüfdaten und native Screenshots.

Native Abnahme: **20/20 SO-101-Fenster mit OpenGL**, 0 verbliebene Widgets;
zusätzlicher Offscreen-Stresstest: **12/12 generische und 20/20 SO-101-Fenster**, ebenfalls
0 verbliebene Widgets und keine Änderung des geschlossenen States durch verspätete Pakete.
[Offscreen-Protokolle](gui-lifecycle-offscreen-verification.json).

## Abschlussprüfung

**137 Tests bestanden, 0 übersprungen, 0 fehlgeschlagen** in 367,32 Sekunden.
Humble, Jazzy und Lyrical führten jeweils den vollständigen 13-Schritt-Task aus.
Auch der native Humble-Lauf, der Versionszyklus und der externe ROS-Szenenemulator
bestanden erneut. Ruff, Formatprüfung, `git diff --check`, lokaler Bootstrap und
`env -u PYTHONPATH .venv/bin/python -m pip check` bestanden.
[Gesamtprotokoll und Paketversionen](regression-verification.json).

Source-Commit der Stabilisierung: `a59c031` — Stabilize Qt object lifetimes and OpenGL context reuse.
Die Docker-Images bleiben installiert; 0 Testcontainer und 0 Testnetze sind übrig.

Die temporäre Socket-ACL wurde nach den Tests vom Benutzer mit
`sudo setfacl -x u:local-user /var/run/docker.sock` entfernt. Die Host-Prüfung bestätigte
den fehlenden benutzerspezifischen Eintrag und `root:docker` mit Modus **0660**.
Der Agentversuch war zuvor an `NoNewPrivs` gescheitert; die Bereinigung ist abgeschlossen.
Die bereits eingerichtete Dockergruppen-Mitgliedschaft bleibt bestehen und erfordert
für neue Prozesse in der Desktop-Sitzung eine neue Anmeldung.
