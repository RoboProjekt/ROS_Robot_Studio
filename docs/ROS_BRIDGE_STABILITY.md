# ROS-Bridge: Fehleruntersuchung und begrenzte Prozessüberwachung

Stand: 8. November 2026. Es wurden ausschließlich eigene, markierte Testcontainer verwendet. Keine globalen Kernel-, Docker-Socket-, Treiber- oder BIOS-Einstellungen wurden für die Untersuchung geändert.

## Beobachteter Fehler und Erkenntnisgrenzen

Ein Humble-`rosbridge_websocket` in Domain 2 endete vor der ersten Clientverbindung mit **Exit-Code −11 / SIGSEGV**. Die anderen Domains blieben funktionsfähig. Die originale Launchdatei ließ `rosapi` und den Launchprozess weiterlaufen, obwohl der WebSocket-Kindprozess bereits beendet war. Ein laufender Container allein war deshalb kein ausreichender Verfügbarkeitsnachweis.

Die Kernelprotokolle ordneten die fehlerhafte Instruktion dem Interpreter `python3.10` zu. Später wurden außerdem native Fehler beim ROS-Setup-Hilfsprogramm bzw. Python-Bootstrap vor der eigentlichen Bridge-Initialisierung beobachtet. In demselben geprüften Zeitraum enthielt das Kernelprotokoll auch unabhängige Browserabstürze. Diese Beobachtungen belegen **keinen Hardwaredefekt** und erlauben keine eindeutige Zuordnung ausschließlich zu Fast DDS oder Studio.

Für die betreffenden UID-10001-Containerfehler lagen nur leere Apport-Dateien vor. Ein auswertbarer ursprünglicher nativer Core-Backtrace war daher nicht verfügbar. Ein vorhandener eigener Host-Python-Crash war ein früherer Qt-Abbaufehler mit SIGABRT; er ist kein Backtrace des ROS-Bridge-SIGSEGV.

Folgende Eingrenzungen wurden tatsächlich vorgenommen:

- **100 isolierte ROS-Setupstarts ohne DDS, WebSocket oder GUI:** alle erfolgreich, mit Prüfung der erzeugten ROS-Umgebung und aktivierter Python-Faultausgabe. [Prüfdaten](ros-bootstrap-diagnostic.json).
- **Python-Paketintegrität:** keine Prüfsummenabweichungen an den geprüften Python-Binär-/Bibliotheksdateien; lediglich optionale Dokumentations-/Manpagedateien fehlen im minimierten Image.
- **Ressourcen:** kein OOM-Kill in den geprüften Stresscontainern, ausreichend freier Datenträgerplatz, keine OOM-Meldung im untersuchten Kernelzeitraum. Die tatsächlichen Speicherwerte je Adapter stehen im Stressbericht; sie lagen im ersten vollständigen Wiederholungslauf unter etwa 107 MB.
- **Netzwerk:** eigene Loopback-WebSocket-Ports, richtige Domain-Umgebung, explizites UDP-Loopback-Profil ohne Host-IPC. Die Pakettypen und identischen Topicnamen wurden mit tatsächlichen ROS-Teilnehmern geprüft. Kein belegter Port bzw. Bind-Fehler wurde als Ursache des SIGSEGV nachgewiesen.
- **Startreihenfolge:** wiederholte überlappende und sequenzielle ROS-Initialisierung mit identischer Domainprüfung. Unerwartete Kindprozessneustarts werden dabei ausdrücklich als Stressfehler gewertet.

**Die ursprüngliche native Fehlerursache ist nicht abschließend identifiziert.** Erfolgreiche Wiederholungen gelten als begrenzter Stabilitätsnachweis und nicht als Beweis, dass ein Upstream-/Umgebungsfehler unmöglich geworden ist.

## Implementierte Fehlerbehandlung

Die eigene Container-Launchdatei `data/runtime/studio_bridge.launch.py` überwacht Bridge und `rosapi`. Ein unerwartetes Prozessende wird mit Exit-Code protokolliert. Jeder dieser Hilfsprozesse darf innerhalb einer Runtime **höchstens zweimal** mit einer Sekunde Abstand neu starten. Danach beendet ein explizites Shutdown-Ereignis die Runtime. Es gibt keinen unbegrenzten Neustartzyklus.

Controller-/Adapterprozesse bleiben zusätzlich durch `entrypoint.sh` und `wait -n` überwacht. Ihr Ausfall beendet die Runtime; sie werden nicht mit einer alten Trajektorie automatisch neu gestartet. Beim normalen Shutdown verhindern der Launch-Shutdownzustand und die eigenen Prozessreferenzen erneute Starts.

`PYTHONFAULTHANDLER=1` gilt auch für den Bootstrap und die Adapter. Der Shell-Fehlerpfad schreibt `STUDIO_RUNTIME_PROCESS_FAILED` mit Exit-Code und Zeile. Ein erneuter Fehler vor dem Launch bleibt damit diagnostizierbar; die Bridge-Neustartlogik kann einen solchen Bootstrapfehler nicht verdecken.

Die Desktopseite erkennt den Verbindungsverlust, invalidiert den State und setzt eine laufende Ausführung auf Fehler. `reconnect_required` sperrt weitere Ausführung und Heartbeats. Der vorhandene Gateway-Watchdog beendet ein ausstehendes Ziel nach ausbleibendem Heartbeat; eine wiederhergestellte WebSocket-Verbindung setzt den Task **nicht** fort. Erst eine ausdrückliche neue Backendzuordnung bzw. Runtime-Neuverbindung mit frischem State ermöglicht neue Planung. Reale Hardware bleibt strikt Read-only.

## Tatsächliche Fehlertests

| Fall | Prüfung |
|---|---|
| A – Normalstart | WebSocket und echte `/rosapi/nodes`-Antwort nach erfolgreicher Initialisierung |
| B – Bridgeausfall | gezieltes Beenden ausschließlich des eigenen Bridge-Kindprozesses; neue PID und wieder verfügbare Discovery |
| C – Wiederholter Ausfall | drei gezielte Kindprozessabbrüche, exakt zwei erfolgreiche Wiederanläufe, danach beendete Runtime |
| D – Bewegungsausfall | echte SO-101-Trajektorie; Bridge während sichtbarer Bewegung beendet; Fehler, ungültiger State, GUI-Meldung und gesperrte automatische Wiederaufnahme |
| E – Backendwechsel | zweite Domain bleibt unverändert; nach dem Ausfall führt sie den vollständigen Task aus, während das ausgefallene Backend keine neuen Ziele erhält |
| F – Cleanup | eigene Container und Netzwerke nicht mehr vorhanden; WebSocket-Port wieder bindbar; keine fremden Instanzen beendet |

Die A–C-/F-Fehlerinjektion läuft unter **Humble, Jazzy und Lyrical**: [Recovery-Bericht](bridge-recovery-verification.json). Die realen D-/E-Prüfungen bestehen ebenfalls unter allen drei Distributionen: [Humble](backend-failure-humble-verification.json), [Jazzy](backend-failure-jazzy-verification.json), [Lyrical](backend-failure-lyrical-verification.json). Der Desktop erkannte die gezielten Verbindungsabbrüche in den geprüften Läufen in deutlich unter einer Sekunde; die tatsächlichen Messwerte stehen in den Berichten.

Der separate [DDS-Stressbericht](backend-stress-verification.json) enthält zwölf Runden mit abwechselnd überlappenden und sequenziellen Starts, insgesamt 36 Adapterstarts in Domains 0/1/2. Er prüft erneut State-/Graph-Isolation, getrennte Endpunkte und schreibgeschützte Command-Spies. **Jeder unerwartete Neustart, native Fehler oder fehlgeschlagene Durchlauf bleibt ein Fehler des Stresstests, auch wenn die Produktionsüberwachung danach wieder eine Verbindung herstellen konnte.** Originalprotokolle werden im ignorierten lokalen `.bootstrap/bridge-stress/`-Cache gesichert und nicht in Git eingecheckt.

Die erste gezielte Recovery-Teilprüfung übersprang Lyrical einmal während des laufenden Image-Builds; nach Buildabschluss wurde die Prüfung für alle drei Distributionen vollständig wiederholt. Fehlgeschlagene Entwicklungszwischenläufe wurden untersucht und zählen nicht als finale Abnahme. Die abschließende vollständige Regression nach der letzten Codeänderung steht im [Integrationsbericht](BACKEND_VERIFICATION.md).

## Wiederholen

```bash
env -u PYTHONPATH .venv/bin/pytest -q tests/test_bridge_recovery.py --run-docker
env -u PYTHONPATH .venv/bin/pytest -q tests/test_backend_integration.py --run-docker --run-isaac
env -u PYTHONPATH QT_QPA_PLATFORM=offscreen .venv/bin/python scripts/verify_backend_stress.py --rounds 12
```

Die Prozessüberwachung verwendet die regulären ROS-Launch-Prozessereignisse; zur ursprünglichen Semantik siehe die [offizielle Humble-Implementierung von ExecuteLocal](https://github.com/ros2/launch/blob/humble/launch/launch/actions/execute_local.py). Die begrenzte Wiederanlaufentscheidung liegt vollständig in Studios eigener Launchdatei.
