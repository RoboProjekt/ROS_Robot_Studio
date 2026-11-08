# ROS-Backend-Plugins entwickeln

Backend-Manifeste liegen in `data/plugins/backend-*.yaml` und verwenden die vorhandene deklarative `PluginRegistry` mit `kind: backend`. `BackendPluginRegistry` ordnet sie `BuiltinROSBackendPlugin` zu. Die Schnittstelle `ROSBackendPlugin` beschreibt Installationsprüfung, Verbindungsanforderungen, Interface-Erkennung und Ausführungsfähigkeiten.

Ein Plugin definiert unterstützte Distributionen und die Ausführungsregel. Fähigkeiten werden aus tatsächlichem Graph, typisiertem frischem Robot State, Controller-Heartbeat und gegebenenfalls Clock abgeleitet. Ein Manifest allein darf keine Bewegung freischalten. Der Manager bleibt für Quellenbindung, aktive Sitzung, Ports, Containerbesitz und Lebenszyklus zuständig.

MuJoCo und Gazebo besitzen Profile, editierbare vorgeschlagene Domains 2/3, Manifeste und klaren Status **planned / Not Implemented**. Sie installieren oder starten keine Simulatoren. Künftige Adapter liefern dieselben JointState-/Controller-/Clock-Verträge; Gazebo kann dafür `ros_gz_bridge` verwenden. Die Task- und Kinematikengine werden dabei nicht ersetzt.

Ein bereits vorhandenes anderes ROS-System kann über Generic ROS beobachtet werden. Das Generic-Profil ist standardmäßig Read-only. Eine explizit bestätigte reine Simulation kann den bestehenden externen Simulationsadapter verwenden. Reale Hardware bleibt dem strikt schreibgeschützten Real-Robot-Profil zugeordnet.

Neue native SDKs gehören in separate Prozesse. Lokale Pfade gehören in Settings, portable Topics/Mapping/Domain/Modellreferenzen ins Projekt. Ergänzungen benötigen Tests für falsche Modelle, fehlende oder alte Rückmeldung, Hintergrundquellen, Abbruch und eigene Ressourcenbereinigung.
