# Zwei getrennte Höhenmodelle mit starker Fisheye-Optik

Diese Trainingsreihe verwendet zwei unabhängig initialisierte Stereo-Tiefenschätzer.
Die Höhe bezeichnet die senkrechte Entfernung des optischen Kamerazentrums zur
Arbeitsfläche. Beide Kameras haben exakt 60 mm Basis und dieselbe Rotation in allen
drei Achsen. Die Blickrichtung des gesamten Halters variiert gemeinsam.

| Modell | Senkrechte Höhe | Modellbild | Disparitätssuchbereich |
| --- | --- | --- | --- |
| High | 42–60 cm | 480 × 288 RGB | 0–127 Pixel |
| Low | 10–20 cm | 320 × 192 RGB | 0–319 Pixel |

Der Winkel wird **gegen die Tischfläche** gemessen, entsprechend
`angle_reference: surface` in den Konfigurationen. 0° bedeutet eine waagerechte
Blickachse, 45° eine schräge Blickrichtung nach unten. Bei sehr flachen Winkeln
liegt der beobachtete Arbeitsbereich unterhalb der Bildmitte. Der Generator hält
ihn dort sichtbar. Die entstehende größere Schrägentfernung darf nicht mit der
senkrechten Montagehöhe verwechselt werden. Die alternative Winkeldefinition
`normal` ist im Generator ausdrücklich konfigurierbar; sie gehört nicht zu diesen
beiden mit `surface` erzeugten Datensätzen.

Die [ausgeführten Messwerte](stereo/height-fisheye/RESULTS.md) dokumentieren
Tiefen-/Objektfehler, Abdeckung, ungesehene Optik, Ressourcen und Beispiele.
Die bisherigen veröffentlichten Modellartefakte werden durch diese beiden
Modelle ersetzt. Der [Modellkatalog](../configs/stereo/model-manifest.json) verweist
auf die getrennten Manifeste. Es gibt kein gemeinsames Standardmodell: die
Variante wird anhand der senkrechten Montagehöhe gewählt.

## Inferenz nach einem frischen Checkout

Beide Gewichte und ONNX-Exporte sind im Repository enthalten:
[High](../checkpoints/so101-high-height-fisheye/README.md) und
[Low](../checkpoints/so101-low-height-fisheye/README.md). Nach Einrichtung der
separaten Python-Umgebung mit `requirements-stereo.lock.txt` lassen sich die
mitgelieferten synthetischen Rohbildpaare ohne Isaac Sim auswerten:

```bash
export STEREO_PY="$PWD/.bootstrap/stereo/venv/bin/python"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2
for profile in high low; do
  "$STEREO_PY" scripts/stereo_depth.py infer --raw-fisheye \
    --checkpoint "checkpoints/so101-${profile}-height-fisheye/best.pt" \
    --left "docs/stereo/height-fisheye/examples/${profile}/left.png" \
    --right "docs/stereo/height-fisheye/examples/${profile}/right.png" \
    --calibration "docs/stereo/height-fisheye/examples/${profile}/metadata.json" \
    --output "training-runs/published-${profile}-example.npz" --device cpu
done
```

## Daten und Optik

Pro Modell werden 615 unabhängige Szenen mit zwei Qualitätsvarianten erzeugt.
420 Szenen / 840 Paare dienen zum Training, 60 / 120 zur Validation, 90 / 180 zum
Test und 45 / 90 zum Test ungesehener starker Linsenparameter. Alle Varianten einer
Szene bleiben im gleichen Split. Die beiden Modelle verwenden unterschiedliche
Seeds und jeweils einen eigenen Datensatz und Optimizer.

Isaac Sim rendert die portable SO-101-Workcell zunächst mit einem weiten
Pinhole-Sichtfeld. Daraus werden zwei native Fisheye-Rohbilder abgebildet und mit
der jeweiligen synthetischen Kalibrierung auf das Modellbild entzerrt. Die
Rekonstruktion verwendet anschließend das zu dieser rektifizierten Ebene passende
`Q`, nicht die Rohbild-Intrinsics. Das neuronale Netz erhält zwei rektifizierte
RGB-Bilder und liefert linke Disparität in Pixeln. Intrinsics und metrische Basis
bleiben separate Größen.

Die Rohkamera hat ein horizontales Fisheye-Sichtfeld von 150–165°. Der
rektifizierte Ausschnitt hat 110°. Im OpenCV-θ-Modell werden je Kamera
`k1 ∈ [-0.08, -0.02]` und `k2 ∈ [-0.003, 0.001]` gezogen. Die Mitte bleibt lokal
nahezu unverzerrt, die äußeren Bildbereiche werden stark komprimiert. Der ungesehene
Satz verwendet zusätzlich `k1 ∈ [-0.13, -0.09]`. Die konkreten Rohbildmatrizen und
Koeffizienten stehen pro Bildpaar in `optics` der Metadaten. Diese Parameter sind
synthetisch und keine Messung der realen AMDIS01B.

Die degradierte Variante erhält Unschärfe σ=0,2–0,8 Rohbildpixel, Rauschen
σ=0,5–2 Grauwerte, Helligkeitsfaktoren 0,95–1,05 und JPEG-Qualität 85/95/100.
Die anschließende starke Randstreckung verstärkt die dortige Unschärfe. Bereiche
ohne fotografische Unterstützung und gemischte Tiefen-/Semantik-Footprints werden
aus der Referenzmaske ausgeschlossen. Die Z-Referenz wird über inverse Tiefe in
die Zielprojektion übertragen, auf ebenen Flächen perspektivisch korrekt.
Das wird analytisch und anhand der gerenderten Tischfläche geprüft.

Objektpositionen beziehen sich weiterhin auf den Median der sichtbaren optischen
XYZ-Punkte einer erodierten Simulator-Maske. Sie sind keine geometrischen
Objektmittelpunkte oder fertigen Greifpunkte. Die kleinen Marker dienen zusätzlich
der beidseitigen Projektions-/Triangulationsprüfung. Tisch und Verdeckungsobjekt
haben im korrigierten Generator getrennte Semantik, damit ein verdeckter Tisch
nicht mit der Oberfläche des Verdeckungswürfels verwechselt wird.

## Ausführung

Aus diesem Checkout starten. `STEREO_PY` muss auf die separate Umgebung mit
`requirements-stereo.lock.txt` zeigen; `ISAAC_PYTHON` auf Isaac Sims eigenes
`python.sh`. Rechnerabhängige Werte stehen nur in der lokalen, ignorierten
`.env` bzw. `training-runs/height-fisheye/local-paths.txt`.

```bash
export STEREO_PY="$PWD/.bootstrap/stereo/venv/bin/python"
export ISAAC_PYTHON="/pfad/zu/isaacsim/python.sh"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2
export CUBLAS_WORKSPACE_CONFIG=:4096:8
unset PYTHONPATH

for profile in high low; do
  "$ISAAC_PYTHON" scripts/generate_stereo_isaac.py \
    --config "configs/stereo/${profile}-height-fisheye.yaml" \
    --output "stereo-data/${profile}-height-fisheye"
  "$STEREO_PY" scripts/audit_stereo_dataset.py \
    --dataset "stereo-data/${profile}-height-fisheye"
  "$STEREO_PY" scripts/stereo_depth.py \
    --config "configs/stereo/${profile}-height-fisheye.yaml" train \
    --dataset "stereo-data/${profile}-height-fisheye" \
    --output "checkpoints/so101-${profile}-height-fisheye" --epochs 80
  "$STEREO_PY" scripts/stereo_depth.py \
    --config "configs/stereo/${profile}-height-fisheye.yaml" train \
    --dataset "stereo-data/${profile}-height-fisheye" \
    --output "checkpoints/so101-${profile}-height-fisheye" \
    --resume "checkpoints/so101-${profile}-height-fisheye/last.pt" --epochs 100
done
```

Für eine neue Erzeugung leere Verzeichnisse verwenden. Bestehende Trainingsläufe
werden nur mit explizitem `--resume` fortgesetzt. Die erste Kapazitätsprüfung
verwendet `train --overfit 4` mit einem separaten Ausgabeverzeichnis. Die GPU führt
Datengenerator und Trainingsprozesse nacheinander aus. BF16 wird nur für den
Trainings-Forward verwendet; Validation, Inferenz und ONNX-Vergleich laufen mit
Float32. Die Gewichte bleiben Float32.

```bash
for profile in high low; do
  for split in test unseen; do
    "$STEREO_PY" scripts/stereo_depth.py \
      --config "configs/stereo/${profile}-height-fisheye.yaml" benchmark \
      --dataset "stereo-data/${profile}-height-fisheye" \
      --checkpoint "checkpoints/so101-${profile}-height-fisheye/best.pt" \
      --output "training-runs/height-fisheye/${profile}-benchmark" \
      --split "$split" --device cuda
  done
done
```

## Gespeichertes Rohbildpaar und Übergabe

Der Datensatz enthält `raw_left`/`raw_right` sowie die bereits rektifizierten
Versionen. Bei einem echten Kamerapaar müssen die tatsächlichen Rohkamera-Intrinsics
und Fisheye-Koeffizienten bekannt sein. Mit einem passenden Metadaten-JSON führt
`infer --raw-fisheye` die kalibrierte Abbildung vor der Schätzung aus:

```bash
profile=low
"$STEREO_PY" scripts/stereo_depth.py unpack --raw \
  --sample "stereo-data/${profile}-height-fisheye/test/test_0031_clean.npz" \
  --output "training-runs/height-fisheye/${profile}-raw-pair"
"$STEREO_PY" scripts/stereo_depth.py infer --raw-fisheye \
  --checkpoint "checkpoints/so101-${profile}-height-fisheye/best.pt" \
  --left "training-runs/height-fisheye/${profile}-raw-pair/left.png" \
  --right "training-runs/height-fisheye/${profile}-raw-pair/right.png" \
  --calibration "training-runs/height-fisheye/${profile}-raw-pair/metadata.json" \
  --output "training-runs/height-fisheye/${profile}-inferred.npz" --device cpu

"$STEREO_PY" scripts/stereo_depth.py export \
  --checkpoint "checkpoints/so101-${profile}-height-fisheye/best.pt" \
  --sample "stereo-data/${profile}-height-fisheye/test/test_0031_clean.npz" \
  --output "checkpoints/so101-${profile}-height-fisheye/compact-stereo.onnx"
```

`LearnedStereo` übernimmt nun den zulässigen optischen Tiefenbereich aus dem
Checkpoint. Dadurch werden nahe Punkte des niedrigen Modells nicht mehr durch
den bisherigen festen 20-cm-Filter verworfen. Die App erwartet weiterhin korrekt
rektifizierte CameraInfo-/TF2-Bildpaare; die neue Roh-Fisheye-Vorverarbeitung ist
ein expliziter Standalone-Schritt. Das bestehende Zeitstempel-Pairing bleibt nötig.

Gültigkeitsmaske und Score sind keine kalibrierte Genauigkeitsgarantie. Sehr
flache Blickrichtungen, unsichtbare Oberflächen und kleine Bildregionen können
zu Ablehnungen oder großen Fehlern führen. Die Referenzmasken im Benchmark stammen
aus dem Simulator. Realaufnahmen, echte Linsenkalibrierung und ein unabhängiger
Realtest sind weiterhin erforderlich.

Der zusammengeführte Bericht und die portablen Modellmanifeste lassen sich aus
den gespeicherten Auswertungen erneut erzeugen:

```bash
"$STEREO_PY" scripts/report_stereo_height.py --output docs/stereo/height-fisheye
```
