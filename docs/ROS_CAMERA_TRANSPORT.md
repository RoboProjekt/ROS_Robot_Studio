# ROS-Kameratransport

Der RGB-Worker führt einen kleinen rclpy-Empfänger im Container der verbundenen Backend-Sitzung aus. Er nutzt deren ROS-Distribution, DDS-Konfiguration und Domain. Der Empfänger liest mit Sensor-Data-QoS und übergibt gerahmte Metadaten und die binären Bildbytes per Docker-Exec-Stream. Der rosbridge-Kanal verarbeitet Graph, Monitoring und Robotik.

- Rohbilder unterstützen `rgb8`, `bgr8` und `mono8` einschließlich Zeilenpadding. Die Kanäle bestehen aus einzelnen Bytes. Beide ROS-Endianness-Angaben sind damit darstellbar.
- `sensor_msgs/msg/CompressedImage` unterstützt JPEG und PNG über Qt. Für alle Formate gelten maximal 1920 × 1080 Pixel und 8 MiB Nutzdaten pro Frame.
- Frame-Puffer halten jeweils das jüngste Bild. Bei einem Rückstau ersetzen neue Bilder wartende Bilder. Bilddekodierung und Inferenz laufen außerhalb des Qt-Hauptthreads.
- CameraInfo und TF liegen im Metadatenpaket des jeweiligen Frames. Die TF2-Abfrage verwendet die ROS-Aufnahmezeit. Rücksprünge der Simulationszeit leeren Bild-, Kalibrierungs- und Ergebniszustand.
- Beim Stoppen schließt Studio den Stream. Eine kurze Lease im Empfänger begrenzt die Lebensdauer auch bei Verlust des Desktop-Prozesses.

Die GUI zeigt Auflösung, Empfangs- und Anzeige-FPS, ersetzte Frames, ROS-Zeitstempel und Wandzeit seit DDS-Empfang. Der Latenzwert beginnt am rclpy-Empfang im lokalen Container. Kamerasensor, Simulationsrendering und DDS-Übertragungszeit vor dem Callback liegen außerhalb dieser Messung.

Bei ausbleibenden Bildern zuerst Domain, Topic-Typ, Optical Frame und Wiedergabezustand prüfen. JPEG oder PNG kann auch bei fehlenden Kalibrierdaten angezeigt werden. Bei ausbleibendem Stream werden Bild und Detektionszustand nach 1,5 s verworfen.
