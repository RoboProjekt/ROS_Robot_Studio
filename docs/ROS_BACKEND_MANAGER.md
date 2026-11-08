# Universal ROS Backend Manager

`ROSBackendManager` verwaltet Profile und gleichzeitige Sitzungen innerhalb der bestehenden Qt-Anwendung. Jede Sitzung besitzt einen eigenen Docker-ROS-Adapter, rosbridge-Transport, Source-Lease, freien Loopback-WebSocket-Port, ROS-Graph und Controllerstatus. Ein gemeinsamer `RobotStateManager` speichert die Zustände getrennt nach Backend-ID. Nur die aktivierte Quelle bestimmt Viewer und Monitoring.

Der vorhandene Taskeditor, die Universal Kinematics Engine und der `ROS2ExecutionBackend` bleiben gemeinsam. Ein Wechsel beendet zuerst die bisherige Ausführung und projiziert danach die neue Quelle. Alte Source-Leases werden zurückgewiesen. Offline aktiviert die lokale Quelle, während andere Verbindungen weiter beobachten können. Disconnect entfernt ausschließlich den jeweiligen Studio-Container und dessen eigenes Bridge-Netzwerk.

`BackendProfile` liegt portabel im Projekt. Modellrevision, Kinematiksignatur und vollständiges URDF→ROS-Mapping binden eine Verbindung an den ausgewählten Roboter. Änderungen am Modell erfordern erneute Zuordnung und Verbindung. Aktive Verbindungen können nicht nebenher umkonfiguriert werden. Fehlende oder alte Rückmeldungen, falsche Nachrichtentypen, fehlende Controller und Clockresets sperren die Ausführung.

Profile und Source-Leases sind getrennt von der aktiven Darstellung. Identische Gelenknamen in zwei Domains überschreiben sich dadurch nicht. Der ausgewählte Backend-Typ allein ist keine Quellenidentität.

Die bisherige Einzelverbindung bleibt für ältere Projekte ohne Backend-Profile kompatibel. Sobald Profile angelegt sind, führt die Runtime-Auswahl über den Backend-Manager. Es gibt keine zweite Taskengine oder zweite Kinematik.

Details: [Domains](ROS_MULTI_DOMAIN.md), [GUI](BACKEND_CONNECTION_MANAGER.md), [Isaac](ISAAC_SIM_INTEGRATION.md), [Prüfungen](BACKEND_VERIFICATION.md).
