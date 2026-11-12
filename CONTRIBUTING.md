# Mitwirken

Entwickle in einem eigenen Branch und beschreibe im Pull Request das geänderte Verhalten und die tatsächlich ausgeführten Prüfungen. Kleine Korrekturen an Bedienpfaden und reproduzierbare Fehlerberichte sind willkommen.

## Lokal prüfen

Starte im Repository mit `./start.sh --demo`. Der Launcher richtet die lokale Python-Umgebung ein. Führe anschließend die normalen Tests und die Formatprüfung aus.

```bash
env -u PYTHONPATH QT_QPA_PLATFORM=offscreen .venv/bin/pytest -q
.venv/bin/ruff check src tests scripts
.venv/bin/ruff format --check src tests scripts
```

Die Integrationstests starten eigene Docker-Runtimes und bei entsprechender Auswahl Isaac Sim. Voraussetzungen, Optionen und Beispielaufrufe stehen in der [Entwicklungsanleitung](DEVELOPMENT.md). Gib für ROS-Fehler Distribution, Domain, Backendtyp und die relevanten Topic-Namen an.

Bevorzugt kleine, für den Test erzeugte URDFs und deklarative Konfigurationen beitragen. Persönliche Pfade, Zugangsdaten, lokale Runtime-Konfigurationen, generierte USD-Dateien und Modellgewichte bleiben außerhalb des Repositorys. Screenshots sollen die tatsächlich ausgeführte Funktion zeigen und eine geeignete Quelle und Lizenz haben.

Änderungen stehen unter der bestehenden [Apache-2.0-Lizenz](LICENSE). Erhalte Drittanbieter-Hinweise und dokumentiere die Herkunft neuer Assets in den [Lizenzhinweisen](THIRD_PARTY_LICENSES.md).
