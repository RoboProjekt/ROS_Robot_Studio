# Backend Connection Manager bedienen

Studio mit `./start.sh --pick-place-demo` öffnen. **Backends…** in der Werkzeugleiste oder **Backend Connections…** im Runtime-Bereich öffnet die Profilverwaltung.

1. **Add** legt ein Profil an. Typ, Distribution und Domain auswählen. Die Vorschläge können geändert werden.
2. Unter **Robot / interfaces** den aktuellen Roboter ausdrücklich zuordnen. Gelenkmapping sowie State-, Command-, Action-, Clock- und TF-Topics prüfen.
3. Für externe DDS-Verbindungen Hostnetz bestätigen. Für lokale Simulatoren Discovery **localhost** wählen. Simulationsausführung verlangt zusätzlich die ausdrückliche Simulationsbestätigung und deaktiviertes Read-only.
4. **Connect** öffnet eine zusätzliche Sitzung. **Set Active** wählt deren frische Rückmeldungen für Viewer und Monitoring. Nur diese Sitzung kann den gemeinsamen Ausführungsadapter benutzen.
5. **Offline demo** schaltet die lokale Quelle aktiv. Hintergrundverbindungen bleiben erhalten. **Disconnect** beendet die gewählte Sitzung.

Editieren, Duplizieren, Löschen sowie Import/Export einzelner YAML-Profile sind vorhanden. Verbundene Profile zuerst trennen; eigene Simulatorprozesse vor Profiländerungen stoppen. Doppelte Domains erfordern die entsprechende ausdrückliche Freigabe. MuJoCo/Gazebo zeigen **Not Implemented** und bieten keinen vorgetäuschten Connect.

**Run Backend Diagnostics** zeigt getrennte PASS/FAIL/WARNING/SKIPPED/NOT TESTED-Befunde. Die Liveansicht enthält Backend-ID, Domain, Distribution, Graph, Modellbindung und tatsächliche Gelenkzustände. Diagnose und Isaac-Prozessprotokoll besitzen eigene Tabs.

Isaac: Connection mode **local_installation** wählen, lokalen Installationspfad auswählen oder erkennen lassen, Domain und JointState-Command-Interface konfigurieren, **Create / validate Isaac scene**, anschließend **Start owned Isaac process**. Der erste Knopf bereitet den Prüfauftrag vor; erst der gestartete Simulator erzeugt bzw. validiert die Szene. **Scene ready** bestätigt diesen Schritt. Danach verbinden und aktivieren. Ein vorhandenes USD kann im Profil ausgewählt werden; Kompatibilitätsbedingungen stehen in [Isaac-Integration](ISAAC_SIM_INTEGRATION.md).

Installationspfad, USD-Pfad, Szenencache und Headless-Einstellung stehen ausschließlich in lokalen Settings unter `backend_local[profile.id]`. Die Projektdatei enthält keine installationsabhängigen absoluten Pfade. Auf einem anderen Rechner dieselben Profile öffnen und die lokalen Pfade neu auswählen. Ohne Isaac bleiben Offline und ROS-Demo verfügbar.
