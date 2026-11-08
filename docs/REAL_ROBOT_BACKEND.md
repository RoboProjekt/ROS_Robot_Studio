# Real Robot: ausschließlich Monitoring

Das Profil schlägt Domain 0 vor, erlaubt aber jede gültige konfigurierte Domain. Es unterstützt ROS-Graph, typisierte JointStates, URDF→ROS-Mapping und Erkennung eines FollowJointTrajectory-Status-Topics. Eine entdeckte Action ist keine Ausführungsfreigabe.

Die Profilvalidierung verbietet `execution_enabled=true` und `read_only=false`. Der Container startet weder Controller noch Action-Gateway. `ReadOnlyRosbridgeTransport` lässt nur Abonnements und festgelegte lesende rosapi-Abfragen zu; Publish, Advertise, Ziele und beliebige Services werden abgewiesen. Auch der gemeinsame ExecutionBackend prüft Read-only erneut.

Aktivieren bedeutet hier, die tatsächlichen Gelenkzustände im Viewer und Live Monitoring zu beobachten. Play bleibt gesperrt. Fehlende Gelenke, ungültige Werte, altes Feedback und widersprüchliche Modellzuordnungen verhindern eine gültige Quelle. Sensor-Efforts werden nur angezeigt, wenn echte Rückmeldungen sie enthalten.

Der Integrationstest verwendet eine reine ROS-Testquelle in Domain 0, keine Roboterhardware. Direkte Publish- und Goal-Versuche werden abgewiesen; ein Command-Subscriber zählt null Befehle. [Messdaten](backend-domain-verification.json). Eine spätere Hardwareausführung benötigt eine eigene ausdrückliche Implementierung und Freigabe; sie ist hier nicht vorhanden.
