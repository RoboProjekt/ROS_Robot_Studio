# Isaac Sim über ROS vorbereiten

## Tatsächlicher Stand

Eine Installation wurde unter `$HOME/isaacsim` gefunden: **5.1.0-rc.19+release.26219**.
Eine laufende, passend konfigurierte SO-101-Szene bzw. ein zugeordnetes USD-Asset liegt
für dieses Studio-Projekt nicht vor (`robot.simulation.asset.path` ist weiterhin null).
Ein echter SO-101-Bewegungstest in Isaac Sim wurde deshalb **nicht durchgeführt**.
Es wurde keine Isaac-Installation verändert und kein Sim-to-Real-Mapping erfunden.

Der ROS-Kommandopfad von `IsaacSimROSAdapter` wurde erfolgreich gegen einen **privaten
ROS-Szenenemulator** getestet: native FollowJointTrajectory-Action → zeitparametrisierte
JointState-Kommandos → beobachtete JointState-Rückmeldung. Das ist ein Adaptertest,
kein Nachweis von Isaac-Sim-Szenenkompatibilität. [Prüfdaten](external-adapter-verification.json).

## Version und Umgebung

Für eine passende Szene sind Humble/Jazzy vorgesehen. Lyrical ist als allgemeine Studio-
ROS-Runtime getestet, seine Isaac-Kompatibilität bleibt unbestätigt. Die Anleitung orientiert
sich an den versionsgebundenen offiziellen [Isaac-Sim-5.1-ROS-Unterlagen](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html)
und [Joint-Control-Beispielen](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_manipulation.html).
Neuere Isaac-Versionen können andere Bibliotheks-/Extensionpfade benötigen; keine Bibliotheken
aus dem Container in die Desktop-.venv oder in Isaacs Python mischen.

## Szene einrichten

1. Einen SO-101 als USD-Articulation bereitstellen. URDF-Jointnamen, Achsen, Nullpose,
   Reihenfolge und Grenzwerte gegen das Originalmodell prüfen. Insbesondere den Greifer
   nicht mit unkalibrierten LeRobot-Motorprozenten verwechseln.
2. Die offizielle ROS-2-Bridge `isaacsim.ros2.bridge` aktivieren. Passende ROS-Umgebung,
   Fast DDS und dieselbe Domain-ID wie im Studio wählen.
3. JointState-Publisher auf das konfigurierte State-Topic legen. Joint-Namen und SI-Werte
   der Articulation liefern. JointState-Subscriber am Command-Topic mit dem Articulation
   Controller verbinden. Efforts nur publizieren, wenn ihre Bedeutung tatsächlich bekannt ist.
4. Simulationsclock sowie nötige TF-Frames publizieren. Play starten und reale Rückmeldungen
   zuerst lesen, bevor ein Bewegungstest freigegeben wird.

Konzeptioneller Action Graph für Isaac 5.1 (Node-Bezeichner gegen die installierte Version prüfen):

```text
On Playback Tick ──exec──► ROS2 Publish Joint State ──target──► SO-101 articulation
       │
       ├──exec──► ROS2 Subscribe Joint State (/joint_command)
       │                   │ jointNames / positionCommand / velocityCommand
       │                   ▼
       └──exec────────► Isaac Articulation Controller ──target──► SO-101 articulation

Isaac Read Simulation Time ──timestamp──► ROS2 Publish Joint State
                         └────────────► ROS2 Publish Clock (/clock)
ROS2 Context (Domain-ID) ──────────────► ROS2 Publisher / Subscriber Nodes
```

Die genaue Verdrahtung, Target-Prims und Articulation-Bindings gehören zur Szene und
wurden hier nicht erzeugt. Kräfte, Kontakt, Greiferstabilität und Kollisionsfreiheit
können aus diesem Graph allein nicht abgeleitet werden.

## DDS zwischen Host und Docker

Im Studio **DDS / Simulation Settings** öffnen, **external_simulation** wählen und die
Simulationsumgebung ausdrücklich bestätigen. Für Host-Isaac/Docker-DDS kann das explizit
bestätigte Hostnetzwerk verwendet werden. Standardmäßig bleibt Studio im Bridge-Netz.
rosbridge bleibt beim Hostprofil an **127.0.0.1** gebunden.

Auf beiden Seiten Domain-ID (Studio-Standard 42) und Middleware `rmw_fastrtps_cpp` setzen.
Das enthaltene `fastdds_udp.xml` aktiviert UDP statt geteilter Speichertransporte. Für den
Host kann dieselbe Datei vor Isaac-Start über `FASTRTPS_DEFAULT_PROFILES_FILE` ausgewählt
werden; den Pfad an die lokale Installation anpassen. Nur eine isolierte Testumgebung
verwenden und sicherstellen, dass keine Hardware dieselbe Domain nutzt.

## Studio verbinden

Unter **Connect to External ROS Simulation** konfigurieren:

- State-Topic, z. B. `/joint_states`.
- Command-Topic, z. B. `/joint_command`.
- Optional vorhandene FollowJointTrajectory-Action oder **joint_state_command**-Adapter.
- `/clock`, `/tf`, Namespace und vollständiges eindeutiges URDF→ROS-Joint-Mapping.
- `use_sim_time`, wenn die Szene eine Simulationsclock publiziert.

Der Adapter hat keine Abhängigkeit von Isaacs Python-API. Er nimmt validierte Cubic-
Trajektorien entgegen, sendet zeitlich abgetastete SI-Kommandos und prüft tatsächliche
Rückmeldungen. Ein publiziertes Kommando ist kein Erfolg. Bei alten Zuständen, fehlender
Clock, Pause/Reset, Zeitlimit oder Verbindungsverlust wird abgebrochen. Ein Clockreset
verlangt erneutes Verbinden und Neuplanung; eine pausierte Szene darf nicht als erfolgreiche
Bewegung gelten. [Offizielle Clock-Konventionen](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_clock.html).

Ein erster realer Test soll ausschließlich eine kleine validierte Simulationbewegung
verwenden, danach Zielzustand und Mapping zurücklesen. Diese Prüfung bleibt bis zum
Vorliegen der konfigurierten SO-101-Szene offen. Hardwareausführung bleibt gesperrt.
