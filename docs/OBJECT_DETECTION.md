# Objekterkennung in Camera & Vision

Nach **Start Stream** einen Detektor auswählen und auf **Start Detection** klicken. Beim Wechsel wird nur der Inferenz-Worker ausgetauscht. Der RGB-Empfänger läuft weiter. Die Anzeige verbindet jedes Overlay mit dem Bild, aus dem es berechnet wurde. **Pause / resume display** friert Bild und sichtbare Ergebnisse ein. **Snapshot…** speichert das dargestellte RGB-Bild als PNG. Für die Auswahl eines Pixels auf einem Detektionsbild in die Vorschau klicken.

| Detektor | Ausgabe |
| --- | --- |
| Color / HSV Detection | Farbregionen, Bounding Boxes, Mittelpunkt und Maskenfüllgrad |
| ArUco Marker Detection | Marker-ID, Bounding Box und vier Bildecken |
| YOLO ONNX | COCO-Klasse, Bounding Box und Modellscore |
| Disabled | RGB-Anzeige |
| Custom Detector… | Vorbereiteter Eintrag für registrierte Implementierungen |

In **Settings…** lassen sich Inferenz-FPS, Klassenfilter, HSV-Intervalle und Mindestfläche sowie das ArUco-Wörterbuch einstellen. Die HSV-Werte verwenden H 0–179 und S/V 0–255. Für Rot sind standardmäßig zwei Intervalle um den Hue-Übergang eingetragen. Maskenfüllgrad und YOLO-Score haben unterschiedliche Bedeutung. ArUco zeigt bei der Erkennung per Wörterbuch die ID und Ecken.

Ein registrierter Detektor implementiert `initialize`, `detect`, `get_classes`, `get_capabilities` und `shutdown` aus `vision.detection.ObjectDetector`. Er erhält ein unveränderliches RGB-Array. Die gemeinsame Worker-Schnittstelle ergänzt Kamera-ID, Backend-ID, Aufnahmezeit und lokale Ergebnis-ID. Sie begrenzt die wartende Eingabe auf ein Bild. Die ID beschreibt das aktuelle Frame-Ergebnis. Eine Objektverfolgung über Frames ist ein eigener zukünftiger Schritt.

[ONNX-Modell verwalten](YOLO_ONNX_INTEGRATION.md) · [Ebene und Pixelbezug](PLANAR_OBJECT_LOCALIZATION.md)
