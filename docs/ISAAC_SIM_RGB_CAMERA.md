# RGB-Kamera in Isaac Sim 5.1

1. Im Runtime-Banner **NVIDIA Isaac Sim** auswählen und die vorhandene SO-101-USD über **Simulation File** öffnen.
2. Einen gegebenenfalls laufenden eigenen Simulator stoppen. In **Sensor Configuration** auf **Add Camera to Isaac Sim…** klicken.
3. Backend, ROS-Topics, TF-Frames, Rendergröße und nominale Render-FPS einstellen. Die Montage **World fixed** verwendet eine Position in Metern und eine USD-Rotation in Grad. **Apply look-at** richtet die optische Achse auf einen Weltpunkt. **Preview pose** zeigt die resultierende Blickrichtung.
4. Optional den roten Testwürfel auf dem Studio-Montagetisch aktivieren. Eine neue USD-Datei angeben. Der separate SDK-Prozess erzeugt sie und meldet den Kameraprim. Studio trägt die abgeleitete Datei ins Runtime-Banner ein.
5. Das Studio-Projekt speichern. **Start Simulation** öffnet die abgeleitete Szene. In Isaac **Play** drücken und in **Camera & Vision** den Stream starten. Bei aktiviertem **Auto Play** beginnt die Szene direkt mit der Wiedergabe.

Der SDK-Prozess verwendet die vorhandene SO-101-Szene und erzeugt eine abgeflachte USD mit gespeichertem Kameraprim und Kamerametadaten. Für jedes Profil konfiguriert er einen Render Product, einen RGB Camera Helper und einen CameraInfo Helper. Der Domain-Kontext entspricht beim Start dem ausgewählten Backendprofil. Der Frame-Skip bezieht sich auf die nominelle Simulationstaktung. Die gemessene Wandzeit-Frequenz hängt von Simulator und GPU ab.

Die statische TF-Kette enthält `world → so101_camera_link → so101_camera_optical_frame` und `world → base_link`. Die Position des Roboters stammt aus dem geladenen USD-Prim. Die optische Achse folgt der ROS-Konvention mit X rechts, Y unten und Z nach vorne. Frames und Topics lassen sich im Assistenten ändern.

Der Studio-Assistent erzeugt feste Weltkameras. Für bewegte USD-Montagen sind die Profiltypen vorbereitet. Szenenbearbeitung läuft in einem separaten Isaac-Prozess. Für eine abgeleitete USD eine freie Zieldatei verwenden. Die visuellen Ergebnisse und Messwerte der Domain-1-Abnahme stehen im Prüfbericht. [NVIDIA Camera Publishing](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_camera_publishing.html) · [Prüfbericht](VISION_VERIFICATION.md)
