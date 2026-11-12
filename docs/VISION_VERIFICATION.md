# RGB-Vision-Abnahme

## SO-101 und Isaac Sim

| Messwert | Ergebnis |
| --- | --- |
| Simulator | Isaac Sim `5.1.0-rc.19+release.26219.9c81211b.gl`, sichtbares X11-Fenster |
| ROS | Humble, Domain 1, Fast DDS |
| Bild | `/so101/camera/image_raw`, RGB8, 640 × 480 |
| Kalibrierung | `/so101/camera/camera_info`, fx = fy = 457,007 px, cx = 320 px, cy = 240 px, `plumb_bob` mit Nullkoeffizienten |
| TF2 | `world → so101_camera_link → so101_camera_optical_frame` und `world → base_link` |
| Kameraprim | `/World/StudioCameras/camera_33ec47e999784df7a060b0d9e500d3d9` |
| Blickrichtung | Welt −Z ab Position (0,230, −0,180, 0,680) m, horizontales FOV 70° |
| Arbeitsebene | Oberseite des Montagetisches bei Welt-Z = −0,005 m mit Normale (0, 0, 1) |
| Bild, Anzeige und HSV | Median 18,16 empfangene FPS, 9,50 Anzeige-FPS und 9,50 Detektionen/s |
| Latenz ab DDS-Callback | 82,4 ms Median, 143,7 ms p95, 139 Stichproben bis zum UI-Sample |
| Begrenzte Frame-Puffer | 190 dekodierte Bilder und 22 ersetzte Bilder im Desktop-Eingang |
| Inferenzdauer | 5,5 ms in der abgebildeten HSV-Stichprobe, 435,9 ms in der abgebildeten YOLO-CPU-Stichprobe |
| Desktop-RAM | 590.472 kB VmRSS zum Messzeitpunkt |
| GUI-Timer | Maximal 59,8 ms zwischen Heartbeats des 25-ms-Timers während des Messfensters |

Studio erzeugte die abgeleitete USD im Kameraassistenten und lud sie nach erneutem
Öffnen des Projekts. Im zunächst pausierten Simulator blieben RGB und Clock aus.
Ein natives Play-Signal startete die Wiedergabe, danach erschienen ROS-Topic, Bild,
passende CameraInfo und TF2. Erneutes Pausieren räumte das angezeigte Bild und die
Detektionen ab. Alle sechs USD-Dateien der Quellszene behielten ihre SHA-256-Werte.

Der HSV-Detektor fand den roten Würfel mit Bounding Box `(285, 244, 324, 277)` und
Maskenfüllgrad 0,622. Die Mitte bei Pixel `(304,5, 260,5)` ergab die Weltposition
`(0,20677, −0,21073, −0,00500) m`. Der Kontaktpunkt aus der USD-Geometrie war
`(0,20800, −0,20300, −0,00500) m`. Die Abweichung betrug **7,8 mm** für die
Bounding-Box-Mitte und **32,5 mm** für die Mitte der Unterkante. Die Transformation
in `base_link` ergab gerundet dieselben Koordinaten, da Welt- und Basisursprung
in dieser Szene praktisch zusammenfallen. Die synthetische Transformationsprüfung
enthält zusätzlich einen versetzten Basisframe.

Im angedockten Workspace bestätigte das vorhandene ROS-Ausführungsbackend eine
Schulterbewegung von **0,10 rad**. Während der Ausführung wurden 61 RGB-Frames und
32 HSV-Ergebnisse verarbeitet. Der Beobachter zählte im Messfenster einschließlich
kurzem Vorlauf 61 JointState-Nachrichten. Das Live Monitoring zeigte den Zustand des
Isaac-Backends. Nach Stream-Stop, Simulator-Stop und GUI-Ende wurden der eigene
Container, der Isaac-Prozess, RGB-Empfänger, Inferenz-Worker und aktive GUI-Timer
freigegeben.

Ein zweiter nativer Lauf bestätigte den 640 × 480-PNG-Snapshot, die Speicherung
der live ermittelten CameraInfo-/TF2-Kalibrierung im Projekt und den Neustart des
Streams. Ein typisiertes `/clock`-Paket setzte die Vision-Epoche zurück. Bild,
Detektionen und Position blieben nach dem Sprung leer. Die zweite Schulterbewegung
lieferte während der Ausführung 46 RGB-Frames, 30 HSV-Ergebnisse und 47 JointStates.
Die Bildfrequenz lag in diesem Lauf bei 14,27 FPS und die Medianlatenz bei 84,5 ms.
[Zweiter SDK-Lauf](../.bootstrap/vision/final-sdk/verification.json) · [Kalibrierungsdialog](../.bootstrap/vision/final-sdk/camera-calibration.png) · [PNG-Snapshot](../.bootstrap/vision/final-sdk/rgb-snapshot.png)

## Zweite RGB-Quelle und Detektoren

Zwei getrennte lokale Humble-Backends in Domain 41 und 42 veröffentlichten auf
dem gleichnamigen Topic `/studio_vision_verification/image/compressed`. Studio
empfing in Domain 41 das Herstellerbild als JPEG mit 810 × 1080 Pixeln. Der
Detektorwechsel von HSV auf YOLO behielt denselben Bildempfänger. Das offizielle
YOLOv5n-v7-Modell erkannte drei Personen mit Scores **0,823**, **0,816**, **0,676**
und einen Bus mit **0,526**. Beim Kamerawechsel wurden Bild und Detektionen geleert.
In Domain 42 erkannte ArUco **Marker 17** im Dictionary `DICT_4X4_50` und lieferte
die vier Bildecken. Beide Prüfsitzungen und ihre Publisher wurden beendet.
[Offizielles YOLOv5-Release](https://github.com/ultralytics/yolov5/releases/tag/v7.0)

Der abschließende Durchlauf benutzte den Dateidialog mit Qt-Maus- und Tastatureingaben.
Die gemessene Inferenzstichprobe lag bei 391,5 ms für YOLO und 40,6 ms für ArUco.
[Abschließende Detektor-Messdaten](../.bootstrap/vision/final-detectors/detector-verification.json)

## Regression

Die vollständige Pytest-Regression mit `--run-docker --run-model-download --run-isaac`
auf `f57fba6` ergab **244 bestanden, 0 fehlgeschlagen, 0 übersprungen**. Humble,
Jazzy, Lyrical, die SO-101-Originaldateien und Isaac waren eingeschaltet. Der
langsame Offscreen-Lebenszyklustest löste nach 180 s eine Watchdog-Ausgabe aus
und schloss anschließend erfolgreich ab. Sieben zusätzliche Tests auf `0bade7b`
prüften Kalibrierungserhalt, Detector-Diagnosen, ungültige Modelle, CameraInfo
und die unmittelbare Freigabe des wartenden Frames. Alle sieben bestanden.

Unter X11 bestanden acht native Bedienzyklen mit OpenGL, Drag-and-drop,
Dateidialogen und Layoutpersistenz. In weiteren 20 SO-101-Fensterzyklen wurden
20 OpenGL-Ansichten und 20 vollständige Fensterfreigaben bestätigt. Der
Lebenszyklustest injiziert ausstehende ROS-Pakete in den Studio-Transport.
[Pytest-Protokoll](../.bootstrap/vision/regression/pytest.log) · [Native Bedienzyklen](../.bootstrap/vision/regression/usability.log) · [SO-101-Fensterzyklen](../.bootstrap/vision/regression/lifecycle.json)

## Lokale Dateien und Start

Die für die Abnahme erzeugte USD liegt unter
`$HOME/Desktop/ROS_APP/ros-robot-studio/.bootstrap/vision/so101-acceptance/SO101_RGB_Vision_Demo.usd`.
Das zugehörige Studio-Projekt und eine lokale Konfiguration mit zugeordneter USD
liegen im selben Ordner. Der Startbefehl nutzt diese Konfiguration. Das Projekt
öffnete sich im geprüften Launcher mit vorausgewählter USD und 17 SO-101-OpenGL-Meshes.
[Studio-Start](../.bootstrap/vision/startup.png)

```bash
cd $HOME/Desktop/ROS_APP/ros-robot-studio
XDG_CONFIG_HOME="$PWD/.bootstrap/vision/so101-acceptance/config" \
  ./start.sh --project "$PWD/.bootstrap/vision/so101-acceptance/SO101_RGB_Vision_Demo.studio.yaml"
```

Im Studio **Start Simulation** wählen, Isaac auf **Play** setzen und dann
**Start Stream** unter **Camera & Vision** klicken. Ein passender Detektor wird
über das Dropdown und **Start Detection** aktiviert.

- [Kameraassistent](../.bootstrap/vision/so101-acceptance/camera-setup-wizard.png), [sichtbare Isaac-Szene](../.bootstrap/vision/so101-acceptance/isaac-camera-scene.png), [RGB-Livebild](../.bootstrap/vision/so101-acceptance/rgb-live.png)
- [HSV und planare Position](../.bootstrap/vision/so101-acceptance/hsv-planar-position.png), [angedockte Ansicht](../.bootstrap/vision/so101-acceptance/camera-beside-robot.png)
- [YOLO](../.bootstrap/vision/detector-acceptance/yolo-onnx.png), [ArUco](../.bootstrap/vision/detector-acceptance/aruco.png)
- [Isaac-Messdaten](../.bootstrap/vision/so101-acceptance/verification.json), [Detektor-Messdaten](../.bootstrap/vision/detector-acceptance/detector-verification.json)

USD, Gewichte, Testbild, Screenshots und Laufzeitprotokolle liegen im lokal
ignorierten Verzeichnis `.bootstrap/vision`. Die Messläufe sind über
`scripts/verify_vision_isaac.py` und `scripts/verify_vision_detectors.py` reproduzierbar.

## Grenzen

Die 7,8 mm gelten für diesen Würfel, diese Blickrichtung und diesen Referenzpixel.
Eine physische Kamera, eine Homographie, bewegte USD-Kameramontagen und weitere
YOLO-Exportverträge stehen aus. Die DDS-Callback-Latenz enthält Transport,
Dekodierung und GUI-Abfrage. Aufnahmezeit und Rendering vor DDS wurden nicht
mitgemessen. Der optionale Vision-Zielentwurf setzt eine explizite Werkzeugorientierung
und einen Versatz voraus.
