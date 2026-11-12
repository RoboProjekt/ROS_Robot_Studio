# RGB-Kamerakalibrierung und Arbeitsebene

Die metrische Lokalisierung nutzt die CameraInfo der aktiven Quelle. Studio prüft Auflösung, Optical Frame, Brennweiten, Hauptpunkt und Verzerrungsmodell. Ein zeitgleiches CameraInfo-Paket hat Vorrang. CameraInfo mit Zeitstempel 0 gilt als statische Kalibrierung. Ein vorausgegangenes gültiges Paket derselben Quelle kann bis zu 500 ms ROS-Zeit weiterverwendet werden. Bei nichttrivialer Verzerrung erfolgt die Entzerrung mit OpenCV.

**Calibrate…** zeigt die gültige Live-Kalibrierung und erlaubt, ihren Snapshot im Kameraprofil abzulegen. Der Snapshot enthält Quelle, Zeitstempel, Auflösung, Intrinsics, Verzerrung und die TF-Pose im Referenzframe. Metrische Live-Schätzungen brauchen weiterhin zum Bild passende Laufzeitdaten. Nach einer Änderung an Kameraoptik oder Kameraaufbau CameraInfo und TF aktualisieren.

Im selben Dialog Name, Frame, einen Punkt auf der Ebene, Normalenrichtung und optional sechs Grenzen (Minimum XYZ, Maximum XYZ) eintragen. Die Ebene anhand der Szene oder einer physischen Messung bestätigen. Die Isaac-Demo übernimmt die Tischoberfläche aus der tatsächlich transformierten USD-Geometrie. Bei einer generischen Szene wird die Ebene vom Nutzer konfiguriert. Den Stream nach Speichern des Profils erneut starten.

Zugeschnittene oder gebinnte Bilddaten brauchen passende, explizit rektifizierte CameraInfo. Unterstützung für andere Verzerrungsmodelle und das Ermitteln neuer Objektivparameter aus einem Kalibriermuster können separat ergänzt werden.
