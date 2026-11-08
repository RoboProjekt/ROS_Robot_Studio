# Docker-Diagnose

**Help → Docker diagnostics** zeigt die Daten des tatsächlich laufenden Studio-Prozesses:
PID, Benutzer/UID, wirksame Prozessgruppen, Kontogruppen, Docker-CLI, Context/Endpoint,
Socketbesitzer/-gruppe/-modus, effektiven Zugriff, Engine-Service und API-Fehler.
**Recheck access** fragt den Zustand erneut im Worker ab. **Setup assistant** öffnet die
bestehende Einrichtung. Wiederholte fehlgeschlagene Starts verwenden dasselbe nichtmodale
Fenster; identische Statusfehler werden nicht wiederholt ins Aktivitätslog geschrieben.
Offline-Modelle, Tasks und Monitoring bleiben nutzbar.

## Ursache auf diesem Rechner

Im sichtbar gestarteten Studio-Prozess war `local-user` laut Kontodaten Mitglied der Gruppe
`docker` (GID 999), die wirksame Gruppenliste des Prozesses enthielt diese Gruppe jedoch
nicht. Docker Engine war aktiv; `/var/run/docker.sock` gehörte `root:docker`, Modus `0660`,
ohne zusätzliche Benutzer-ACL. Die API antwortete mit `PermissionError(13)`.
Das ist eine **alte Prozess-/Desktop-Sitzung**, keine fehlende Kontomitgliedschaft.
[Messung aus dem sichtbaren Studio](gui-usability-stale-session.json),
[Diagnosefenster](docker-diagnostics-stale-session.png).

Vollständig vom Desktop abmelden, erneut anmelden und Studio aus dieser neuen Sitzung
starten. Erneutes `usermod` repariert einen bereits laufenden Prozess nicht. Für eine
bewusst gestartete neue Gruppensitzung kann ein bereits berechtigter Benutzer im normalen
Terminal aus dem Projektverzeichnis auch Folgendes verwenden:

```bash
sg docker -c './start.sh --pick-place-demo'
```

Genau eine solche neue Sitzung konnte hier API und alle drei vorhandenen Studio-Images
erreichen. Dabei wurden weder Mitgliedschaften noch Socketrechte verändert. `start.sh`
erhöht seine Rechte nicht selbst. Die Docker-Gruppe hat weitreichende Hostrechte; eine
neue Mitgliedschaft bleibt eine getrennt zu prüfende Systementscheidung.

## Unterscheidbare Zustände

| Diagnose | Bedeutung / nächster Schritt |
|---|---|
| `process_groups_inactive` | Konto gehört zur Socketgruppe, dieser Prozess noch nicht: neue Anmeldung/Sitzung |
| `permission_denied` | Zugriff verweigert trotz anderer Gruppenlage: Socket/ACL/Endpoint prüfen |
| `not_installed` | CLI fehlt und API nicht erreichbar: Setup prüfen |
| `daemon_stopped` | Engine-Service nachweislich inaktiv/fehlgeschlagen: Dienst starten |
| `socket_missing` | Gewählter Unix-Socket fehlt: Engine und Endpoint prüfen |
| `invalid_context` | Context nicht auflösbar: gültigen Context wählen |
| `api_unreachable` | Transport/API nicht erreichbar; kein pauschaler Gruppenhinweis |

`DOCKER_CONTEXT` hat Vorrang vor `DOCKER_HOST`; sonst gilt der gespeicherte CLI-Context.
Rootless Unix-Sockets und konfigurierte Remote-Endpunkte werden berücksichtigt. Der SDK-
Client verwendet denselben aufgelösten Endpoint; ein anderer CLI-Context wird nicht still
ignoriert. TLS-Dateien bleiben lokal. Ein vorhandener Socket allein beweist keine laufende
Engine. Ein erreichbarer Daemon beweist wiederum kein installiertes Studio-Image und keine
bereite ROS-Bridge. Imageinstallation, Containerstart, Healthcheck und ROS-Rückmeldung sind
weiterhin getrennte Schritte mit eigenen Fehlermeldungen.

Die Diagnose führt keine `chmod`-, ACL-, Gruppen-, Dienst- oder sudo-Änderungen aus.
