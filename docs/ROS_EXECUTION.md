# ROS-Simulationsausführung

`motion.ros_backend.ROS2ExecutionBackend` verwendet die vorhandenen `CompiledTask`- und
`JointTrajectory`-Objekte. Es gibt keine zweite IK oder Bewegungsplanung. Nur die Profile
`managed_simulation` und ausdrücklich bestätigte `external_simulation` sind zugelassen.
`hardware_ready` bleibt false und Hardwareausführung im Projekt fest gesperrt.

## Datenfluss

```text
TaskCompiler → vorhandene Cubic-JointTrajectory → TrajectoryToROSConverter
  → rosbridge /studio/execution/command (begrenzter JSON-Vertrag in std_msgs/String)
  → containerinterner rclpy ActionClient
  → control_msgs/action/FollowJointTrajectory
  → generischer SimulationController
  → /joint_states → RobotStateManager → beide Viewer / Monitoring
```

Actionverarbeitung liegt im ROS-fähigen Container. Desktop und Gateway tauschen
Session-ID, Modellsignatur, Goal-ID und typisierte Ereignisse aus. Kontrollereignisse
werden nicht durch das Latest-Sample-Throttling normaler Messwerte verworfen. GUI-Kommandos
werden bei Verbindungsverlust nicht gepuffert oder automatisch erneut gesendet.

## Konvertierung und Controller

`TrajectoryToROSConverter` erzeugt `trajectory_msgs/msg/JointTrajectory` mit Jointnamen,
Positionen, analytischen Geschwindigkeiten/Beschleunigungen und relativen Nanosekundenzeiten.
Die Cubic-Kurve wird zeitlich abgetastet, nicht neu geplant. Der Simulator verwendet
Hermite-Interpolation mit denselben Endwerten und Ableitungen; dadurch bleibt die empfangene
Cubic-Kurve erhalten. Shared-Interpolation und URDF-Parser werden in das Image kopiert.

`runtime.controller_core.SimulatedTrajectory` prüft Namen, vollständige Arrays, endliche
Werte, streng steigende Zeiten, Startabweichung, URDF- und Mimic-Grenzen sowie analytische
Extrema der Position/Geschwindigkeit/Beschleunigung jedes Hermite-Intervalls.
Ungültige Goals werden abgelehnt. Der rclpy-Server publiziert reale ROS-JointStates mit
berechneten Simulationpositionen/-geschwindigkeiten, akzeptiert Cancel und liefert Feedback/Result.
Efforts werden nicht erfunden. Die unveränderte SO-101-URDF enthält sechs unabhängig
bewegliche Joints einschließlich Greifer; dieselbe Implementierung funktioniert generisch.

## Sequenz und Status

Vor Start sind nötig: gültiger vollständiger Plan, passendes Modell/Mapping, frisches
JointState-Feedback und beobachteter Controller. Der erwartete Trajektorienstart wird
mit dem tatsächlichen Zustand verglichen. Jeder Schritt muss erfolgreich beendet sein,
bevor der nächste startet. Joint/Cartesian/Relative/Home nutzen dieselben Gelenkraumkurven;
Greifer öffnen/schließen sind validierte Gelenktrajektorien. Wait hält zwischen bestätigten
Schritten, während weiterhin ROS-Feedback empfangen wird.

Status: not_connected, connected, ready, executing, succeeded, failed, cancelled, timeout.
Publish allein ist kein Erfolg. Erforderlich sind Goal Acceptance, erfolgreicher Action-
Resultcode **und** beobachtete Endwerte innerhalb der Toleranz. Timeout, fehlendes Feedback,
veralteter Zustand, Ablehnung, falsche Startpose und Verbindungsabbruch stoppen die Ausgabe.
Das Gateway überwacht einen GUI-Heartbeat und fordert bei dessen Ausfall Cancel an.

Ein ROS-Task kann mit Stop abgebrochen werden. Pause/Resume einer laufenden ROS-Action
ist in diesem Stand nicht implementiert und wird ausdrücklich abgewiesen. Offline-Pause
bleibt verfügbar. Nach Fehler/Verbindungsverlust ist explizites Disconnect/Reconnect und
Neuplanung nötig; keine automatische Fortsetzung. Lokaler Reset überschreibt kein ROS-Feedback.

Der virtuelle Würfel bleibt eine visuelle Kopplung. Im ROS-Modus folgen Pose und
abgeschlossene Attach-/Release-Ereignisse dem beobachteten Zustand und den bestätigten
Actionfortschritten; keine Kontakt-/Kraftsimulation.

## APIs und Erweiterungen

Qt benötigt seine eigenen QObject.connect/disconnect-Methoden; das ROS-Backend verwendet
`connect_backend`/`disconnect_backend`. Hinzu kommen `execute`, `abort`, `read_state`,
`read_joint_positions` und ein explizit verweigerter unvalidierter Direkt-Greiferbefehl.
`OfflinePreviewBackend` behält seine bisherige API. Beide erhalten dieselben kompilierten Tasks.

`IsaacSimROSAdapter` kann optional standardisierte JointState-Kommandos zeitlich ausgeben
und Erfolg nur aus beobachteten Rückmeldungen ableiten. Konfiguration und tatsächlicher
Teststatus stehen in [ISAAC_SIM_ROS_SETUP.md](ISAAC_SIM_ROS_SETUP.md).

**Collision checking not available – preview only.** ROS-Simulation ist weder
Kollisionsprüfung noch Hardwarefreigabe. Ein ROS-Cancel ist kein physischer Not-Halt.
