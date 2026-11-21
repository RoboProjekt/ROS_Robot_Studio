# Trainierter SO-101-Stereo-Tiefenschätzer

Dieser lokale Entwicklungsstand liefert einen tatsächlich trainierten, kompakten Stereo-Schätzer und einen ausgeführten Vergleich mit StereoSGBM. Alle Genauigkeiten stammen aus Isaac Sim. Für die beiden späteren Conceptronic AMDIS01B liegen noch keine Realaufnahmen oder gemessenen Intrinsics vor.

## Repository und geprüfte Umgebung

Der saubere Veröffentlichungscheckout verfolgt `git@github.com:RoboProjekt/ROS_Robot_Studio.git`, Branch `main`, Upstream `origin/main`. Vor Quelländerungen wurde `git pull --ff-only` ausgeführt. Ausgangscommit war `b3887e6fdba6734f97aa23dffadd43046d629f06`, aktualisierter Basiscommit ist `902e7f82cb718463be8649134982786c77662089`. Die Entwicklung erfolgt lokal auf `stereo-depth-training`. Der ältere Desktop-Checkout ohne Remote wurde erhalten.

| Komponente | Tatsächlich verwendet |
| --- | --- |
| System | Ubuntu 22.04, Kernel 6.8.0-149-generic, Ryzen 7 5800X, 32 GB RAM |
| GPU | NVIDIA GeForce RTX 4070 Ti, 12.282 MiB VRAM, Treiber 580.178.04 |
| Simulator | Isaac Sim 5.1.0-rc.19+release.26219.9c81211b.gl, Python 3.11 SDK |
| Training | Python 3.10.12, PyTorch 2.6.0+cu124, CUDA-Runtime 12.4 |
| Klassische Auswertung | OpenCV Contrib Headless 4.12.0.88 |
| Export | ONNX 1.17.0, ONNX Runtime 1.22.1, Opset 17 |

Es wurden keine Systemberechtigungen geändert. Simulator und Training liefen nacheinander. BLAS-/Torch-Threads sind begrenzt. Die Trainingsumgebung ist von Desktop-App und Simulator getrennt. Die vollständigen aufgelösten Versionen stehen in `requirements-stereo.lock.txt`.

## Szene und Datenvertrag

`scripts/generate_stereo_isaac.py` lädt `Projects/SO101_Stereo_Workcell.usda` mit den vorhandenen Studio-Kameranamen, Arbeitsfläche, rotem Würfel und blauer Schale. Hinzu kommen ein Referenzwürfel mit Farbtextur, eine prozedurale Tischtextur, ein grüner Verdeckungswürfel und vier bekannte gelbe Referenzpunkte. Der portable Fixture-Layer enthält keinen vollständigen gerenderten Roboter. Der Workflow benötigt weder ROS/Docker noch Qwen, Ollama, MoveIt oder die GUI.

Je Bildpaar werden eine NPZ-Datei und JSON-Metadaten geschrieben. NPZ enthält RGB links/rechts als `uint8 HWC`, optische Z-Tiefe in Metern, linke Disparität in Pixeln, boolesche Gültigkeitsmaske und sichtbare semantische Masken beider Kameras. JSON enthält Szene, Frame, Seed, Zeitstempel, `K`, `P`, `Q`, Kameraposen, metrische Basis, optische Frames, Objekt-IDs und Referenzpunkte. Weltkoordinaten sind Z-oben; `so101_work_surface` verschiebt die ursprüngliche Tischhöhe −0,005 m nach Z=0.

Der Replicator-Annotator `distance_to_image_plane` liefert optische Z-Tiefe. Radiale Entfernung wird nicht als Z verwendet. Die Kameras sind parallele Pinholes ohne Verzeichnung. Render-Pixelzentren liegen bei `cx=(W−1)/2`, `cy=(H−1)/2`. Die Disparität ist `u_links−u_rechts`, positive Tiefe folgt aus `Z=fx·B/(d−cx_links+cx_rechts)`. Die Rekonstruktion verwendet `Q` und berücksichtigt auch ungleiche Hauptpunkte. Einheiten- und Resize-Tests decken diesen Fall ab. Die dichte Gültigkeit verlangt Sichtbarkeit im rechten synchronen Bild, gültige Bildränder und einen Z-Abgleich mit 3 mm Toleranz. Zeitversetzte Tests behalten die Referenz des linken Aufnahmezeitpunkts.

Der Objektreferenzpunkt ist der **koordinatenweise Median von optischem XYZ** auf der um einen Pixel erodierten linken sichtbaren Simulator-Maske. Genau dieser Punkt wird starr in Welt- und Arbeitsflächenkoordinaten transformiert. Er bezeichnet eine sichtbare Oberfläche, keinen geometrischen Objektmittelpunkt und keinen bereits bestimmten Greifpunkt. Beide Verfahren verwenden dieselben Masken und dieselbe Referenzregel. Die Masken kommen im Benchmark aus dem Simulator; eine Detektorqualität wird damit nicht gemessen. Der App-Adapter behält separat die bestehenden HSV-Masken und deren 5×5-Erosion bei.

| Split | Unabhängige Szenen | Bildpaare |
| --- | ---: | ---: |
| Train | 360 | 720 |
| Validation | 60 | 120 |
| Test | 90 | 180 |
| Ungesehene Kamerakonfigurationen | 45 | 90 |
| Zeitversatz, gepaarte Zustände | 36 | 216 |
| Gesamt | 591 | 1.326 |

Alle Qualitätsvarianten und Zeitversätze einer Szene bleiben im gleichen Split. Train/Validation/Test verwenden 60/80/100 mm Basis und 30/45/60 cm nominalen Arbeitsabstand. Verdeckung variiert unabhängig von den neun Basis-/Abstandskombinationen. Es gibt Montageverschiebungen und neue Blickrichtungen mit eigener Kalibrierung, Würfeltextur oder glatte Flächen, Randlagen, variable Beleuchtung, Kontrast, Gaußunschärfe σ=0/0,8/1,6 und JPEG-Qualität 100/85/65. Unschärfe und Kompressionsstufe werden unabhängig ausgewählt; die Kontraständerung gehört zur degradierten Gruppe. Die Schale bleibt eine überwiegend glatte, unterschiedlich helle blaue Fläche.

Der ungesehene Satz verwendet 120 mm Basis, 72° horizontales Sichtfeld und 33/57 cm Abstand. In jeder der 36 Bewegungsszenen werden **alle** Zeitversätze 0/33/100 ms mit derselben Pose und Kalibrierung gerendert. Roter und Referenzwürfel bewegen sich tatsächlich mit 0,3 m/s zwischen linkem und rechtem Zustand, also um 0/9,9/30 mm. Die Schale bleibt stehen. Diese Fälle sind vom Training ausgeschlossen.

Vor der Massengenerierung wurden zwölf Szenen visuell und numerisch kontrolliert. Der endgültige Generator prüft vor den weiteren Aufnahmen beide Bildprojektionen und die analytische Tischfläche. `audit_stereo_dataset.py` prüft zusätzlich alle Paare, Einheiten, Q-Rekonstruktion, Referenzframes, Split-Trennung und die vollständigen Zeitversatzgruppen und schreibt SHA-256-Prüfsummen aller NPZ-/JSON-Dateien. Teilweise verdeckte oder abgeschnittene kleine Kugeln verschieben den sichtbaren Pixelmittelpunkt gegenüber der projizierten Kugelmitte; die maximalen Markerabweichungen sind deshalb kein reiner Kalibrierfehler. Die numerischen Audit-Ergebnisse stehen im [Messbericht](stereo/verification.json).

## Modell und Reproduktion

`CompactStereo` ist eine hier implementierte Architektur mit **11.210 trainierbaren Parametern** unter Apache-2.0, ohne vortrainierte Fremdgewichte. Ein geteilter Encoder erzeugt Merkmale bei Viertelauflösung. Ein horizontales Kostenvolumen aus Merkmals- und RGB-Differenzen wird mit kleinen 3D-Faltungen regularisiert, per Softargmin ausgewertet und im RGB-Bild verfeinert. 320×192 und maximal 128 Pixel Disparität halten Speicher und Laufzeit klein. Dies ist ein synthetischer Prototyp mit einem überschaubaren Trainingslauf, kein umfassend auf reale Webcam-Bilder trainiertes Modell.

Eingang ist ein gleich großes, rektifiziertes **RGB-Paar**, `uint8 HWC` beim Python-Adapter bzw. `float32 NCHW [1,3,192,320]` bei ONNX. Normalisierung ist ausschließlich Division durch 255. Vor dem Modell wird bilinear auf 320×192 skaliert, ohne äußeres Padding. Interne Faltungen verwenden Zero-Padding. Ausgabe sind linke Disparität in Pixeln dieser Auflösung und ein Score aus der maximalen groben Softmax-Wahrscheinlichkeit. Für die ursprüngliche Auflösung wird die Karte bilinear skaliert und die Disparität mit `Originalbreite/320` multipliziert. Anschließend gilt das originale kalibrierte `Q`. Bei explizitem Resize der Kalibrierung gilt für Pixelzentren `c'=(c+0,5)·scale−0,5`.

Der Gültigkeitsschwellwert ist 0,08, hinzu kommen positive Disparität und rechte Bildgrenzen. Dieser Score ist **keine kalibrierte Fehlerwahrscheinlichkeit**. Der Benchmark begrenzt die Z-Tiefe für beide Verfahren auf 0,18–1,0 m. Objektpositionen benötigen mindestens 16 gültige Punkte und 10 % Maskenunterstützung, mit höchstens 35 mm medianer Z-Abweichung. Ablehnungsgründe werden gezählt.

Seed ist 101. AdamW verwendet Lernrate 0,001, Weight Decay 0,0001, Batch 8, Smooth-L1 auf gültiger Referenzdisparität, fünffaches Objektpixelgewicht und Gradient Clipping 10. Es gibt keine Verwendung der Testdaten zur Auswahl. Der beste Checkpoint wird ausschließlich nach dem Validierungs-EPE bestimmt. `last.pt` enthält zusätzlich Optimizer, Epochennummer und Python-/NumPy-/Torch-/CUDA-Zufallszustände. Die Szene-Splits, Konfiguration, Verlauf und Datenfingerprint werden gespeichert. Train-/Validierungsdateien werden gegen die Audit-Prüfsummen geprüft. Ein Regressionstest vergleicht unterbrochenes und durchgehendes Training auf bitgleiche Gewichte. Deterministische Ergebnisse sind für diese Umgebung angestrebt; andere GPU-/Treiber-/Renderer-Versionen können numerisch abweichen.

Die endgültigen Zahlen, Fehlergruppen, Laufzeiten, Artefaktprüfsummen und Beispiele stehen im [ausgeführten Messbericht](stereo/RESULTS.md). Auf ausdrücklichen Wunsch werden die beiden kleinen geprüften Modellartefakte mit veröffentlicht: [best.pt](../checkpoints/so101-compact-stereo/best.pt) (ca. 171 KiB) und [compact-stereo.onnx](../checkpoints/so101-compact-stereo/compact-stereo.onnx) (ca. 205 KiB). Der Datensatz, der fortsetzbare lokale Stand `last.pt`, Trainingsausgaben und Rechnerpfade bleiben außerhalb der Git-Historie. Das [Modellmanifest](../configs/stereo/model-manifest.json) enthält die SHA-256-Prüfsummen und beschreibt Format und Nachverarbeitung. Der ursprüngliche Export erzeugt daneben ein nur lokal gespeichertes Manifest mit absolutem Checkpoint-Pfad.

Für eine Inferenzprüfung nach einem frischen Checkout steht ein [kleines synthetisches RGB-Beispielpaar](stereo/example/metadata.json) mit Kalibrierung bereit. Nach Einrichtung der separaten Python-Umgebung funktioniert dieser Aufruf ohne Isaac Sim und ohne Datensatzerzeugung:

```bash
.bootstrap/stereo/venv/bin/python scripts/stereo_depth.py infer \
  --checkpoint checkpoints/so101-compact-stereo/best.pt \
  --left docs/stereo/example/left.png --right docs/stereo/example/right.png \
  --calibration docs/stereo/example/metadata.json \
  --output training-runs/published-example.npz --device cpu
```

## Ausführbare Startbefehle

Aus dem aktualisierten Repository ausführen. Für einen neuen Lauf ein **leeres** Datensatz- und Ausgabeverzeichnis wählen. Die hier ausgeführten lokalen Artefakte verwenden bereits `stereo-data/so101` und `checkpoints/so101-compact-stereo`.

```bash
python3.10 -m venv .bootstrap/stereo/venv
.bootstrap/stereo/venv/bin/python -m pip install \
  -r requirements-stereo.lock.txt \
  --extra-index-url https://download.pytorch.org/whl/cu124
export PY="$PWD/.bootstrap/stereo/venv/bin/python"
export ISAAC_PYTHON="/pfad/zu/isaacsim/python.sh"
export DATASET="$PWD/stereo-data/so101"
export RUN="$PWD/checkpoints/so101-compact-stereo"
export RESULTS="$PWD/training-runs/so101"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2
export CUBLAS_WORKSPACE_CONFIG=:4096:8
unset PYTHONPATH
```

Isaac verwendet sein eigenes Python. Die Trainingspakete werden dort nicht installiert. Die verwendete lokale Installation wird im nicht versionierten `training-runs/so101/local-paths.txt` festgehalten. Für eine CPU-Umgebung müssen Torch-Wheel und `train.device` bewusst angepasst werden.

```bash
# Optional zwölf Vorschau-Szenen, dann separat der vollständige Datensatz
"$ISAAC_PYTHON" scripts/generate_stereo_isaac.py \
  --config configs/stereo/so101.yaml --preview --output stereo-data/preview
"$ISAAC_PYTHON" scripts/generate_stereo_isaac.py \
  --config configs/stereo/so101.yaml --output "$DATASET"
"$PY" scripts/audit_stereo_dataset.py --dataset "$DATASET"

# Kleine Kapazitätsprüfung, danach der ausgeführte Hauptlauf und seine Fortsetzung
"$PY" scripts/stereo_depth.py train --dataset "$DATASET" \
  --output "$RESULTS/overfit" --overfit 4 --epochs 180
"$PY" scripts/stereo_depth.py train --dataset "$DATASET" --output "$RUN" --epochs 60
"$PY" scripts/stereo_depth.py train --dataset "$DATASET" --output "$RUN" \
  --resume "$RUN/last.pt" --epochs 80

for split in validation test unseen asynchronous; do
  "$PY" scripts/stereo_depth.py benchmark --dataset "$DATASET" \
    --checkpoint "$RUN/best.pt" --output "$RESULTS/benchmark" \
    --split "$split" --device cuda
done

# Gespeichertes Bildpaar und originale kalibrierte Pixelauflösung
"$PY" scripts/stereo_depth.py unpack \
  --sample "$DATASET/test/test_0001_clean.npz" --output "$RESULTS/inference-pair"
"$PY" scripts/stereo_depth.py infer --checkpoint "$RUN/best.pt" \
  --left "$RESULTS/inference-pair/left.png" --right "$RESULTS/inference-pair/right.png" \
  --calibration "$RESULTS/inference-pair/metadata.json" \
  --output "$RESULTS/inferred.npz" --device cpu
"$PY" scripts/stereo_depth.py export --checkpoint "$RUN/best.pt" \
  --sample "$DATASET/test/test_0001_clean.npz" --output "$RUN/compact-stereo.onnx"

PYTHONPATH=src "$PY" -m pytest --noconftest -q tests/test_stereo_learning.py

"$PY" scripts/plot_stereo_run.py --run "$RUN" \
  --benchmark "$RESULTS/benchmark" --output "$RESULTS/plots"
```

Die YAML-Datei steuert die Basisprofile `baselines_m`, Abstände, Auflösung, Szeneanzahl und Trainingsparameter. `baseline_m` ist der nominale Startwert bzw. Fallback ohne Profilliste. Für ein einzelnes Basis-/Abstandsprofil sind zusätzlich `generate_stereo_isaac.py --baseline-m 0.08 --distance-m 0.45` möglich; die ausdrücklich ungesehene Testgruppe behält ihre andere Konfiguration. `mount_jitter_deg` steuert eine kleine näherungsweise Winkelvariation über Positions-/Zielverschiebungen. Intrinsics und Posen werden immer neu gespeichert.

## Ergänzte milde Kameratests vom 21.11.2026

Der [zusätzliche Blur-/Linsenbericht](stereo/robustness-mild/RESULTS.md) vergleicht neun Profile auf jeweils denselben 90 sauberen Testpaaren. Konfigurierbar sind leichte Gauß- und Bewegungsunschärfe, unterschiedliche Schärfe links/rechts, Brown-Conrady- und Fisheye-Verzeichnung sowie eine kleine Fehlkalibrierung rechts. Der kombinierte Fall ergänzt geringes Rauschen und einen Helligkeitsunterschied. Die Standardverzeichnung verschiebt Punkte um höchstens 4,27 Pixel bei 320×192; die größte Restabweichung nach der absichtlich fehlerhaften rechten Entzerrung beträgt 1,07 Pixel.

```bash
"$PY" scripts/stereo_depth.py robustness \
  --dataset "$DATASET" --checkpoint "$RUN/best.pt" \
  --effects-config configs/stereo/robustness-mild.yaml \
  --output "$RESULTS/robustness-mild" --device cuda

PYTHONPATH=src "$PY" -m pytest --noconftest -q \
  tests/test_stereo_learning.py tests/test_stereo_camera_effects.py
```

Ein neues Ausgabeverzeichnis wählen, wenn der Bericht dort bereits existiert. Die originalen Trainingsdateien und Gewichte bleiben erhalten. Der Vergleich verwendet für alle Profile dieselbe fotografisch verfügbare Referenzregion. Die linke Kamera wird immer geometrisch korrekt entzerrt; die rechte erhält im Kalibrierfehlertest leicht unpassende Parameter. Damit bleiben linke Tiefenlabels und Q konsistent. Diese Erweiterung betrifft Offline-Tests. Live-Fisheye-CameraInfo in der GUI ist damit noch nicht ergänzt.

## Übergabe an Robot Studio

Der Austauschpunkt bleibt `OpenCVStereo.compute_disparity()`. Das unveränderte `estimate()`-Ergebnis enthält rektifizierte RGB-Bilder, Disparität, metrische Punkte und Transformation. `LearnedStereo` erbt CameraInfo-Prüfung, bildzeitbezogene TF2-Transformation, OpenCV-Rektifizierung, Q-Rekonstruktion und das bestehende HSV-Objektergebnis. Zusätzlich weist es Paare über dem vorhandenen Studio-Limit von 25 ms Zeitversatz ab. Das schützt nicht vor einer unbekannten Differenz zwischen USB-Aufnahmezeitpunkt und gemeldetem Zeitstempel. Der bestehende Pairer prüft außerdem Alter und Reihenfolge.

```python
from functools import partial
from ros_robot_studio.gui.stereo_workspace import StereoWorkspace
from ros_robot_studio.vision.stereo_learning.adapter import LearnedStereo, estimate_objects

workspace = StereoWorkspace(
    vision, estimator_factory=partial(LearnedStereo, checkpoint_path, device="cpu")
)
# RGBFrame-Objekte enthalten optischen frame_id, stamp_ns, CameraInfo und Bildzeit-TF2.
objects = estimate_objects(
    checkpoint_path, left_rgb_frame, right_rgb_frame, "so101_work_surface", device="cpu"
)
# Je Objekt: object_id, xyz, reference_frame, timestamp_ns, valid, reference_kind.
```

Torch bleibt eine optionale Trainings-/Adapter-Abhängigkeit. Für den Desktop muss es ausdrücklich in einer kompatiblen Umgebung verfügbar sein. Der normale Studio-Schätzer benötigt kein Torch. ONNX wurde exportiert und numerisch geprüft; ein ONNX-basierter GUI-Adapter ist nicht Teil dieses Austauschpunkts.

Für den Realtest müssen beide AMDIS01B in ihrer tatsächlichen Betriebsauflösung gemeinsam kalibriert, Rektifizierungsfehler und physischer Basisabstand geprüft und die Zeitstempel gegen Bewegung kontrolliert werden. Danach folgt ein getrennt protokollierter Test mit gemessenen Referenzabständen und Objektpunkten. Synthetische Linsen, ideale Posen und Simulator-Masken belegen noch keine reale Greifgenauigkeit.

## Primärquellen

- [Isaac Sim 5.1 Camera API und optische Tiefe](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.sensors.camera/docs/index.html)
- [NVIDIA Replicator](https://docs.omniverse.nvidia.com/kit/docs/omni_replicator/latest/index.html)
- [OpenCV Kalibrierung, stereoRectify, triangulatePoints und reprojectImageTo3D](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)
- [Offizielle PyTorch-Wheel-Installation](https://pytorch.org/get-started/locally/)
