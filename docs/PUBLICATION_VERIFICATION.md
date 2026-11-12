# Dokumentations- und Freigabeprüfung

Stand 13. November 2026. Studio-Version 0.1.0. Die README enthält eine Funktionsübersicht und ein bebildertes deutsches SO-101-Tutorial mit den englischen GUI-Bezeichnungen.

## Frischer Checkout

Ein separater lokaler Git-Clone startete mit leeren XDG-Verzeichnissen und einer neuen `.venv`. `./start.sh --smoke-test` und `pip check` bestanden. Der Modellmanager lud die gepinnte SO-101-Revision neu, prüfte 16 Quelldateien und öffnete acht Links, sieben Joints und 17 Visual-Geometrien. SciPy wurde separat installiert. Die Humble-Abnahme verwendete das vorhandene Studio-Docker-Image und bestätigte einen nativen Action-Task mit 13 Schritten, 789 JointState-Samples, konsistente Positionen und Geschwindigkeiten sowie Abbruch und Moduswechsel.

## Native Bedienung und USD

- In X11 ein Projekt mit dem Namen **SO-101 Lab** angelegt, die Original-URDF über den Qt-Dateidialog importiert und das Katalogmodell aktiviert.
- **Chain setup** mit `base_link`, `gripper_frame_link` und `gripper` bestätigt. Den Task **SO-101 shoulder check** mit Joint-Ziel, Wait und Home im Schrittdialog erstellt und validiert.
- Mit **Generate from current robot** aus der ursprünglichen URDF eine USD erzeugt. Die Scene-Validation bestätigte Stage Units, Originalgelenke und Quellenprüfsummen. Unter Humble in Domain 1 bestätigte das JointState-Feedback das Schulterziel und die Rückkehr zur Nullpose. Das Studio-Projekt wurde gespeichert und mit zugeordneter USD erneut geöffnet.
- In der Isaac-SDK-Prüfung den generierten Stage geöffnet, den echten ROS-Action-Graph inspiziert und eine eigene USDA mit Lichtintensität 900 aus der Basis mit 800 exportiert und erneut geöffnet. Die Prüfsumme der Basis blieb identisch.
- Die generierte USD als Quelle für die RGB-Ableitung wiederverwendet. CameraInfo, TF2, HSV, Snapshot, gespeicherte Kalibrierung und die parallele SO-101-Bewegung bestanden. Die absolute Genauigkeit hängt von Kamera, Ebene und gewähltem Pixel ab. [Vision-Abnahme](VISION_VERIFICATION.md)
- YOLOv5n v7 erkannte im JPEG-ROS-Teststream die Klasse `stop sign` mit Score 0,917. Das Testmotiv wurde eigens für die Dokumentation gezeichnet. ArUco erkannte Marker 17 in einem separaten DDS-Backend.

## Regression und Freigabeumfang

Die gezielte Regression für GUI, Robotermodelle, Kinematik, Taskeditor, Backend-Verwaltung und Vision ergab **109 bestanden**. Die vorherige vollständige Regression mit allen ROS-Distributionen und Isaac sowie die 28 nativen Fensterzyklen sind im [Vision-Bericht](VISION_VERIFICATION.md#regression) dokumentiert.

Der Quellcode steht unter Apache-2.0. Der SO-101-Download enthält die Upstream-Lizenz und Quellenhinweise. Simulationsdateien, Modellgewichte, das Herstellerfoto der früheren YOLO-Abnahme und persönliche Einstellungen liegen lokal. [Drittanbieter-Hinweise](../THIRD_PARTY_LICENSES.md)

Gitleaks fand bei der Prüfung der bestehenden Historie keine Zugangsdaten. Die Prüfung erkannte personenbezogene Metadaten in historischen Commits und Aufnahmen. Die aktuelle Dokumentation verwendet neutrale Beispielpfade und geeignete Bildausschnitte.

## Bildverzeichnis

Die Hauptbilder, Funktionsansichten und Tutorialbilder stammen aus tatsächlichen Studio- oder Isaac-Fenstern. Die Dokumentationsausschnitte zeigen die ursprünglichen Bedienelemente. Der YOLO- und ArUco-Teststream ist im Tutorial als solcher bezeichnet.

### Übersicht

- [robot-studio-main.png](images/overview/robot-studio-main.png)

### Funktionsansichten

- [camera-vision.png](images/features/camera-vision.png)
- [dockable-workspace.png](images/features/dockable-workspace.png)
- [kinematics.png](images/features/kinematics.png)
- [live-monitoring.png](images/features/live-monitoring.png)
- [robot-configuration.png](images/features/robot-configuration.png)
- [simulation-runtime.png](images/features/simulation-runtime.png)
- [task-editor.png](images/features/task-editor.png)

### Tutorial

- [00-setup-assistant.png](images/tutorial/00-setup-assistant.png)
- [01-create-project.png](images/tutorial/01-create-project.png)
- [02-urdf-picker.png](images/tutorial/02-urdf-picker.png)
- [03-urdf-imported.png](images/tutorial/03-urdf-imported.png)
- [04-so101-loaded.png](images/tutorial/04-so101-loaded.png)
- [05-joint-controls.png](images/tutorial/05-joint-controls.png)
- [06-chain-setup.png](images/tutorial/06-chain-setup.png)
- [07-solver-selection.png](images/tutorial/07-solver-selection.png)
- [08-ros-runtime.png](images/tutorial/08-ros-runtime.png)
- [09-backend-manager.png](images/tutorial/09-backend-manager.png)
- [10-isaac-profile.png](images/tutorial/10-isaac-profile.png)
- [11-generate-scene.png](images/tutorial/11-generate-scene.png)
- [12-isaac-stage.png](images/tutorial/12-isaac-stage.png)
- [13-isaac-articulation.png](images/tutorial/13-isaac-articulation.png)
- [14-isaac-joint.png](images/tutorial/14-isaac-joint.png)
- [15-isaac-ros-graph.png](images/tutorial/15-isaac-ros-graph.png)
- [16-isaac-ros-context.png](images/tutorial/16-isaac-ros-context.png)
- [17-isaac-scene-edited.png](images/tutorial/17-isaac-scene-edited.png)
- [19-simulation-paused.png](images/tutorial/19-simulation-paused.png)
- [20-simulation-ready.png](images/tutorial/20-simulation-ready.png)
- [22-live-monitoring.png](images/tutorial/22-live-monitoring.png)
- [23-teach-in.png](images/tutorial/23-teach-in.png)
- [25-task-action.png](images/tutorial/25-task-action.png)
- [26-task-completed.png](images/tutorial/26-task-completed.png)
- [27-camera-setup.png](images/tutorial/27-camera-setup.png)
- [28-isaac-camera-stage.png](images/tutorial/28-isaac-camera-stage.png)
- [28-rgb-stream.png](images/tutorial/28-rgb-stream.png)
- [29-detector-dropdown.png](images/tutorial/29-detector-dropdown.png)
- [29-hsv-settings.png](images/tutorial/29-hsv-settings.png)
- [30-detection-result.png](images/tutorial/30-detection-result.png)
- [31-calibration.png](images/tutorial/31-calibration.png)
- [32-aruco.png](images/tutorial/32-aruco.png)
- [33-yolo-onnx.png](images/tutorial/33-yolo-onnx.png)
