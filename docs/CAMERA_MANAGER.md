# RGB-Kameras verwalten

Unter **Sensor Configuration → Cameras** zeigt Studio Name, Backend, Domain, Topics, Optical Frame, Kalibrierung und Streamstatus. **Add Camera…** legt ein Profil für ein bestehendes ROS-Backend an. Nach der Backend-Verbindung erscheinen im editierbaren RGB-Feld erkannte `Image`- und `CompressedImage`-Topics. Das Feld akzeptiert auch einen manuell eingegebenen Topic-Namen. Bei manueller Eingabe den passenden Transport wählen.

**Connect camera** verbindet das ROS-Backend. **Open Camera View → Start Stream** abonniert dessen Bilddaten. **Disconnect camera** beendet den Kameraempfang und gibt dessen Abonnements frei. Die Backend-Sitzung lässt sich in **Backend Connections** separat beenden. Das erlaubt beispielsweise weiterlaufendes JointState-Monitoring.

Ein Projekt kann mehrere Kameraprofile enthalten. Innerhalb eines Backends sind Bild-/CameraInfo-Topics und die beiden Kameraframes eindeutig. Verschiedene Backends dürfen gleichnamige Topics verwenden. Die Kameraauswahl beendet den bisherigen Empfänger und löscht dessen Bild, Overlays und Positionsentwürfe. Anschließend den neuen Stream starten. Die GUI zeigt jeweils eine aktive RGB-Quelle.

Kameraprofile werden mit dem Studio-Projekt gespeichert. Die Domain stammt aus dem referenzierten Backendprofil. Installation, lokale USD-Datei und ONNX-Modellpfade werden in den lokalen Einstellungen geführt. Auf einem zweiten Rechner werden diese Dateien erneut zugeordnet. Bestehende Projekte öffnen mit leerer Kameraliste.

[Isaac-Kamera erstellen](ISAAC_SIM_RGB_CAMERA.md) · [Transport und Diagnosen](ROS_CAMERA_TRANSPORT.md)
