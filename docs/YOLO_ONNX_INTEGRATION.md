# YOLO ONNX

Unter **Camera & Vision → Add ONNX…** ein lokales Modell registrieren. **YOLO ONNX** und das Modell im Dropdown auswählen und die Inferenz starten. Der Modellmanager speichert lokale Pfade. **Remove ONNX** entfernt den Eintrag aus Studio. Die Datei bleibt auf dem Datenträger.

Der geprüfte Modellvertrag ist **YOLOv5n v7.0 COCO, fester 640 × 640 RGB-Input, Batch 1**. Der Adapter akzeptiert NCHW FP16 oder FP32 und rohe Ausgaben der Form `[1,25200,85]` mit 80 Klassenlabels in den ONNX-`names`-Metadaten. Ausgabeform, Datentyp und Labels werden beim Laden geprüft. Das offizielle FP16-Release lief mit ONNX Runtime auf der CPU. Andere YOLO-Generationen, dynamische Exportformen und ONNX-Graphen mit integrierter NMS sind außerhalb dieses Modellvertrags.

Der Worker skaliert mit Letterbox, normalisiert RGB auf 0–1, berechnet den Score aus Objektwahrscheinlichkeit mal Klassenwert, führt klassenweise NMS aus und projiziert Boxen zurück in die ursprüngliche Bildgröße. Score-Grenze und Klassenfilter stehen unter **Settings…**. Die Bildquelle kann jede unterstützte RGB-Quelle sein.

Die Abnahme nutzte das separat heruntergeladene offizielle `yolov5n.onnx` und das Herstellerbild `bus.jpg`. Beide Dateien werden lokal bereitgestellt. Der Ultralytics-Quellstand v7.0 ist GPL-3.0. [Release v7.0](https://github.com/ultralytics/yolov5/releases/tag/v7.0) · [Exportcode](https://github.com/ultralytics/yolov5/blob/v7.0/export.py) · [Lizenz v7.0](https://github.com/ultralytics/yolov5/blob/v7.0/LICENSE)
