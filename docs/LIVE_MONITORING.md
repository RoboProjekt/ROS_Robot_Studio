# Live Monitoring

**Robot State Monitoring** ist die Standardansicht aller Roboterprojekte. Die frühere
Sinusdemo liegt ausschließlich in der expliziten zweiten Ansicht **Synthetic Signal
Demo / ROS messages**. Öffnen des Monitoring-Tabs startet keine eigene Roboteranimation.

Die Hauptansicht zeigt Modell/Revision, aktives Backend, Quelle, gültig/veraltet,
Zeitstempel, Empfangsalter, Aktualisierungsrate und Ausführungsstatus. Die Tabelle wird
aus der URDF aufgebaut: Joint, Position, Geschwindigkeit, Effort und Status. Eine kompakte
3D-Ansicht verwendet dieselben State-Snapshots wie Robot Configuration.

- Regleränderung: neue Position und FK; Geschwindigkeit/Effort N/A.
- Übernommene IK: neue virtuelle Pose; eine Berechnung allein ändert nichts.
- Offline-Task: Position, Geschwindigkeit und Beschleunigung aus derselben Cubic-Kurve.
- ROS: Name/Position/Velocity/Effort aus echten JointState-Nachrichten; keine lokale Ersatzanimation.

Die Feldliste bietet `position.<joint>`, `velocity.<joint>`, optionale Efforts,
Beschleunigungen, `tool.x/y/z/roll/pitch/yaw` und `execution.progress`. Bis zu acht Kurven
werden gleichzeitig angezeigt. Nicht gelieferte Größen erzeugen keine erfundenen Nullkurven.
Die Tabelle weist SI-Einheiten aus; bei gemischten Kurven bleiben die Feldnamen in der Legende sichtbar.

**Pause Plot** friert die Kurvenaufzeichnung ein, nicht den Roboter. **Resume Plot** setzt
sie mit aktuellen Daten fort. **Clear Plot** leert nur den Puffer. Das Zeitfenster ist
zwischen 5 und 300 s einstellbar. Maximal 6.000 Samples begrenzen den Speicherbedarf.
Metadatenupdates erzeugen keine zusätzlichen Messpunkte mit gleichem Zeitstempel.

Offline verwendet eine monotone lokale Zeit. Bei ROS werden gültige Header-Stamps
verwendet, sonst die monotone Empfangszeit. Die X-Achse zeigt Sekunden vor dem letzten
Sample. Quellen-/Modellwechsel beginnen einen neuen Plotkontext. Bei zurückgesetzter
Quellzeit wird der Plot zurückgesetzt; eine laufende externe Bewegung muss neu verbunden/geplant werden.

In **Synthetic Signal Demo / ROS messages** bleiben Graph-, Topic-/Node-Suche, generische
Message-Inspektion und die ausdrücklich synthetischen Sinus-/Temperaturdaten verfügbar.
Auch das lokale `/joint_states` enthält dort den ausgewählten Roboterzustand, keine
fest verdrahteten generischen Armwerte. ROS-TF/Pose/String/Trajectory-Nachrichten werden
generisch inspiziert; große numerische Strukturen sind begrenzt.

Das Projekt speichert `monitoring.source`, `robot_fields`, Fenster und die bisherigen
Topic-/Feldeinstellungen. Alte Projekte werden mit Robot-State-Monitoring geöffnet.

Native Screenshots und tatsächlich ausgeführte Tests: [ROS_RUNTIME_VERIFICATION.md](ROS_RUNTIME_VERIFICATION.md).
