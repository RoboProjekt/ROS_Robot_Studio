# RGB-Kamera in Isaac Sim 5.1

1. Im Runtime-Banner **NVIDIA Isaac Sim** auswählen und die vorhandene SO-101-USD über **Simulation File** öffnen.
2. Einen gegebenenfalls laufenden eigenen Simulator stoppen. In **Sensor Configuration** auf **Add Camera to Isaac Sim…** klicken.
3. Backend, ROS-Topics, TF-Frames, Rendergröße und nominale Render-FPS einstellen. Die Montage **World fixed** verwendet eine Position in Metern und eine USD-Rotation in Grad. **Apply look-at** richtet die optische Achse auf einen Weltpunkt. **Preview pose** zeigt die resultierende Blickrichtung.
4. Optional den roten Testwürfel auf dem Studio-Montagetisch aktivieren. Eine neue USD-Datei angeben. Der separate SDK-Prozess erzeugt sie und meldet den Kameraprim. Studio trägt die abgeleitete Datei ins Runtime-Banner ein.
5. Das Studio-Projekt speichern. **Start Simulation** öffnet die abgeleitete Szene. In Isaac **Play** drücken und in **Camera & Vision** den Stream starten. Bei aktiviertem **Auto Play** beginnt die Szene direkt mit der Wiedergabe.

Der SDK-Prozess verwendet die vorhandene SO-101-Szene und erzeugt eine abgeflachte USD mit gespeichertem Kameraprim und Kamerametadaten. Für jedes Profil konfiguriert er einen Render Product, einen RGB Camera Helper und einen CameraInfo Helper. Der Domain-Kontext entspricht beim Start dem ausgewählten Backendprofil. Der Frame-Skip bezieht sich auf die nominelle Simulationstaktung. Die gemessene Wandzeit-Frequenz hängt von Simulator und GPU ab.

Die statische TF-Kette enthält `world → so101_camera_link → so101_camera_optical_frame` und `world → base_link`. Die Position des Roboters stammt aus dem geladenen USD-Prim. Die optische Achse folgt der ROS-Konvention mit X rechts, Y unten und Z nach vorne. Frames und Topics lassen sich im Assistenten ändern.

Der Studio-Assistent erzeugt feste Weltkameras. Für bewegte USD-Montagen sind die Profiltypen vorbereitet. Szenenbearbeitung läuft in einem separaten Isaac-Prozess. Für eine abgeleitete USD eine freie Zieldatei verwenden. Die visuellen Ergebnisse und Messwerte der Domain-1-Abnahme stehen im Prüfbericht. [NVIDIA Camera Publishing](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_camera_publishing.html) · [Prüfbericht](VISION_VERIFICATION.md)

## Stereo und Qwen mit einem separaten Isaac-Rechner

Isaac Sim 5.1 läuft als eigener Prozess auf dem Simulationsrechner. Auf dem Studio-Rechner laufen die beiden ROS-Empfänger, OpenCV, die GUI und Ollama. Die Tiefenschätzung verwendet eine austauschbare `estimate(left, right, base)`-Schnittstelle. Ein später trainiertes Modell kann diese Schnittstelle implementieren. Studio benötigt dafür weder die Trainingsumgebung noch einen laufenden Trainingsprozess.

### USD vorbereiten und starten

Die geprüfte USD-Arbeitszelle liegt unter `Projects/SO101_Stereo_Workcell.usda`. Sie enthält Tisch, strukturierten roten Würfel, blaue Schale und beide Kameraprims. Die kombinierte Datei mit Roboter und ROS-Publishern wird auf dem Isaac-Rechner als `Projects/SO101_Stereo_Demo.usda` erzeugt.

- Auf dem Isaac-Rechner im Studio-Banner ein **Isaac Sim**-Profil mit **Humble**, **Domain 1** und **Local installation** wählen. Die vorhandene SO-101-USD unter **Simulation File** auswählen. Als Ausgangspunkt kann Studio auch die katalogisierte SO-101-URDF importieren.
- In **Camera & Vision → Stereo & Planning** den Basisabstand und den Arbeitsabstand einstellen. Standard sind **80 mm** und **45 cm**. Der Arbeitsabstand ist zwischen **30 und 60 cm** einstellbar.
- **Build Derived Isaac USD…** öffnen und im Projektverzeichnis `Projects/SO101_Stereo_Demo.usda` speichern. Im Runtime-Banner **Start Simulation** und in Isaac **Play** wählen.
- Auf dem Studio-Rechner ein aktives Isaac-Profil für die vorhandene oder entfernte Simulation in **Humble, Domain 1** verwenden. Für die Erkennung über das lokale Netzwerk **Host** und **Subnet** wählen. Das Studio-Banner verbindet dieses Profil mit der laufenden Simulation. **Register Stereo Cameras** registriert die Standardtopics für dieses Profil. **Start Stereo** startet beide Empfänger.

Auf einem Linux-Isaac-Rechner lässt sich die Anfrage auch vom Terminal vorbereiten. Ersetze `SO101_URDF`, `SO101_SOURCE` und `ISAAC_HOME` durch die dortigen Pfade. Wenn die alte Szene fehlt, entfällt `--source`.

```bash
request=$(PYTHONPATH=src .venv/bin/python scripts/prepare_so101_stereo.py \
  "$SO101_URDF" "$PWD/Projects/SO101_Stereo_Demo.usda" --source "$SO101_SOURCE")
env -u PYTHONPATH -u PYTHONHOME -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
  ROS_DISTRO=humble ROS_DOMAIN_ID=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  ROS_LOCALHOST_ONLY=0 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
  FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/ros_robot_studio/data/runtime/fastdds_udp.xml" \
  LD_LIBRARY_PATH="$ISAAC_HOME/exts/isaacsim.ros2.bridge/humble/lib" \
  "$ISAAC_HOME/python.sh" "$PWD/src/ros_robot_studio/data/isaac/studio_scene.py" \
  --request "$request" --auto-play
```

| Sicht | RGB `sensor_msgs/msg/Image` | Kalibrierung `sensor_msgs/msg/CameraInfo` | Optischer Frame |
|---|---|---|---|
| Links | `/so101/stereo/left/image_raw` | `/so101/stereo/left/camera_info` | `so101_stereo_left_optical_frame` |
| Rechts | `/so101/stereo/right/image_raw` | `/so101/stereo/right/camera_info` | `so101_stereo_right_optical_frame` |

Der Graph veröffentlicht `world → so101_stereo_{left,right}_link → so101_stereo_{left,right}_optical_frame`, `world → base_link → robot_base` und `world → so101_work_surface` auf `/tf_static`. `robot_base` ist ein identischer Alias des MoveIt-Planungsframes `base_link`. Die Bildempfänger fragen TF2 zum jeweiligen Aufnahmezeitpunkt ab. **Left**, **Right** und **Stereo / Disparity** verwenden Bildpaare mit höchstens 25 ms Zeitdifferenz. Jeder Puffer hält höchstens vier Frames. Pause, Uhr-Rücksprung, Backendwechsel und Frames älter als 750 ms verwerfen die aktuelle Szene.

Studio zeigt die gemessenen XYZ-Oberflächenpunkte in Metern. Die App benutzt beim Würfel die bekannte Kantenlänge von 55 mm und die gemessene Tischreferenz, um daraus einen Greifmittelpunkt abzuleiten. Der Arbeitsflächenrahmen ist im Panel sichtbar. **Links bedeutet +Y in `robot_base`**. Die für das Training vorgesehenen AMDIS01B benötigen ihre eigene Kalibrierung. Die Stereotiefe hier stammt aus rektifizierten Bildern und StereoSGBM. [Isaac Sim 5.1 CameraInfo und RGB](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_camera.html)

### Auftrag, MoveIt und Vorschau

1. Im Stereo-Panel **Check Ollama** wählen. Standard sind `http://127.0.0.1:11434` und `qwen3-vl:8b-instruct`. Studio fragt den Dienst vom Desktop-Prozess aus ab. Für Studio in einem Container erreicht dessen Loopback den Host nur mit Host-Netzwerk. Ansonsten die erreichbare Hostadresse als Basis-URL eintragen.
2. Einen Auftrag wie „Den roten Würfel links neben die blaue Schale legen“ eingeben und **Interpret Task** wählen. Die App sendet den Auftrag, das beschriftete RGB-Bild und den zugehörigen Snapshot an `/api/chat`. Das Modell wählt Aktion, Objekt-IDs und Relation. Studio prüft die Antwort gegen das Schema und bestimmt Greifpose und Ablagekoordinaten. Für die Öffnungsbewegung am Ablagepunkt plant Studio 12 mm Werkzeugabstand zum berechneten Objektmittelpunkt ein.
3. **Plan Only** löst die Studio-Kinematik und fragt MoveIt in Humble, Domain 1 ab. **Preview Path…** zeigt die Trajektorien des Bildauftrags im Modell. Bei fehlender Tiefe, veralteter Beobachtung, verändertem Roboterzustand oder fehlgeschlagener Planung bleibt der Pfad gesperrt.

Der separate Planungsdienst heißt standardmäßig `studio-so101-moveit`. Seine SRDF definiert `arm` als Kette von `base_link` zu `gripper_frame_link` und `gripper` als Gruppe für das namensgleiche Gelenk. Die App prüft beide Gruppen und die laufende Planning Scene. Tisch, Schale und der mitgeführte Würfel sind vereinfachte Kollisionskörper. Die Trajektorienausführung ist in dieser Konfiguration abgeschaltet. [Ollama Chat API](https://docs.ollama.com/api/chat) · [Qwen-Modell](https://ollama.com/library/qwen3-vl:8b-instruct)

```bash
docker build -f docker/Dockerfile.moveit-humble -t ros-robot-studio/humble-moveit:0.1.0 .
.venv/bin/python scripts/configure_so101_moveit.py "$SO101_URDF" "$PWD/.local/so101-moveit"
docker run --rm --name studio-so101-moveit --network host -e ROS_DOMAIN_ID=1 \
  --mount "type=bind,source=${SO101_URDF%/*},target=/studio_robot,readonly" \
  --mount "type=bind,source=$PWD/.local/so101-moveit,target=/studio_config,readonly" \
  --entrypoint bash ros-robot-studio/humble-moveit:0.1.0 -c \
  'source /opt/ros/humble/setup.bash; exec ros2 run moveit_ros_move_group move_group --ros-args --params-file /studio_config/moveit.yaml'
```

### Abnahme auf dem Studio-Rechner am 20. November 2026

| Prüfung | Ergebnis |
|---|---|
| Git und vorhandene Software | Aktiver Klon unter `~/Desktop/ROS_Robot_Studio`, Ausgangsstand `599e2b9`. Humble-Image mit TF2 und rosbridge. MoveIt wurde in einem separaten Humble-Image ergänzt. |
| Zwei ROS-Bilder und TF2 | Zwei tatsächlich abonnierte Humble-Bildtopics in Domain 1, mit synthetischer Quelle und separaten Testframes. |
| 30 cm | 300,0 mm aus StereoSGBM |
| 45 cm | 450,0 mm aus StereoSGBM |
| 60 cm | 600,0 mm aus StereoSGBM |
| Fehlerfall | Rechtsbild ohne Korrespondenz. Würfel-ID blieb sichtbar und seine Position ungültig. Die Planung blieb gesperrt. |
| Mock | Gültiger Vorschlag und Rückfrage angenommen. Unbekannte ID und ungültiges JSON abgelehnt. |
| Echtes Qwen | Ollama-Server 0.40.16. Modell `70e58e37beb1` verfügbar. Der frühere Pull-Blocker verhindert den lokalen Bildtest nicht mehr. Qwen hat `red_cube`, `blue_bowl` und `left_of` ausgegeben. `ollama ps` zeigte 5,8 GB, 100 % GPU und 4096 Kontext auf der RTX 3060. |
| MoveIt Plan Only | MoveIt 2.5.10, OMPL, `so101_new_calib`, Gruppen `arm` und `gripper`. Planungsdienste `/get_planning_scene`, `/apply_planning_scene` und `/plan_kinematic_path` erfolgreich geprüft. Mit dem validierten echten Qwen-Vorschlag entstanden acht Phasen. Ablagepunkt in Metern `[0.1793, 0.1485, 0.0225]`. |
| Isaac und Controller | Der separate Isaac-Rechner war für diese Sitzung nicht erreichbar. Der USD-Arbeitszellen-Layer wurde mit OpenUSD geöffnet. Der vollständige Isaac-Renderpfad und die simulierte Controller-Ausführung bleiben für den zweiten Rechner offen. In Domain 1 war kein `FollowJointTrajectory`-Controller sichtbar. |
| Greifer | Öffnen, Schließen und Loslassen wurden als MoveIt-Gelenkziele geplant. Physischer Objektkontakt ist vor Ort separat zu prüfen. Der Orchestrator hat keinen Zugriff auf den realen SO-101. |

Die Distanzmessungen stammen aus idealen, parallel ausgerichteten Testbildern. Für den Bildtest wurde zusätzlich ein synthetischer Tisch-Snapshot mit eigenen festen Extrinsiken verwendet. Beide Abnahmen verwenden dieselben Desktop-Komponenten wie der Isaac-Empfang.
