# ROS-Domains und Netzwerke

Vorschlagswerte stehen ausschließlich in `data/backends/defaults.json`:

| Typ | Domain | Umsetzung |
|---|---:|---|
| Real Robot | 0 | Nur Beobachtung |
| Isaac Sim | 1 | Externe Simulation |
| MuJoCo | 2 | Vorbereitetes Plugin |
| Gazebo | 3 | Vorbereitetes Plugin |
| ROS Demo | 10 | Verwalteter Simulationscontroller |
| Generic ROS | 0 | Standardmäßig nur Beobachtung |

Alle Werte sind editierbar. `ROSDomainManager` prüft Ganzzahlen 0…232 anhand der RTPS-Portformel (Basis 7400, Domainabstand 250). Er warnt vor dem tatsächlichen lokalen Ephemeral-Portbereich und verlangt für mehrfach verwendete Domains eine ausdrückliche Freigabe. Domain-IDs sind Routing und keine Hardwareberechtigung oder Sicherheitsgrenze.

Jedes ROS-Backend läuft als eigener Adapterprozess im Container, mit eigener ROS-Umgebung. Das Desktop-Python lädt kein rclpy und setzt keine globale ROS-Domain. Steuerkanäle tragen zusätzlich eine Backend-ID, Sitzung und Modellsignatur. WebSocket-Ports werden lokal reserviert und nach dem Start weiterhin pro Sitzung nachverfolgt.

Die ROS-Demo erhält ein eigenes verwaltetes Docker-Bridge-Netz. Externe lokale/entfernte DDS-Verbindungen verwenden nach Bestätigung das Hostnetz; rosbridge lauscht dabei ausschließlich auf 127.0.0.1. Container laufen als Benutzer 10001, ohne zusätzliche Capabilities, mit `no-new-privileges` und Ressourcenlimits. Es werden weder Host-IPC noch privilegierte Container benötigt.

Für lokale Simulatoren ist `discovery=localhost` vorgesehen. `fastdds_localhost.xml` begrenzt UDP explizit auf 127.0.0.1 und verwendet lokale Unicast-Peers. `ROS_LOCALHOST_ONLY=0` verhindert, dass Humble zusätzlich SHM einschaltet. Das ist nötig, weil die Adapter eigene IPC-Namensräume haben. Für externe Rechner aktiviert `discovery=subnet` das allgemeine UDP-Profil. Beide Seiten müssen Domain, Middleware und erreichbare Netzwerkschnittstellen passend konfigurieren.

Technischer Hintergrund: [Humble-Implementierung der Teilnehmerkonfiguration](https://github.com/ros2/rmw_fastrtps/blob/humble/rmw_fastrtps_shared_cpp/src/participant.cpp). Die tatsächliche Domain-Prüfung verwendet drei Containerpaare im selben Hostnetz mit identischen Topicnamen; Ergebnisse in [backend-domain-verification.json](backend-domain-verification.json).
