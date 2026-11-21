# SO-101-Stereo: 10–20 cm, starke Fisheye-Optik

Dieses eigenständig trainierte CompactStereo-Modell ersetzt zusammen mit der
anderen Höhenvariante das frühere gemeinsame Stereo-Modell. Beide Varianten
wurden für 100 Epochen trainiert; der gewählte beste Checkpoint liegt hier bei
Epoche 94. Die Messungen stammen ausschließlich aus Isaac Sim.

- Senkrechte Sensorhöhe: **10–20 cm**; Kameraabstand **60 mm**.
- Relative Rotation: **0° in allen drei Achsen**.
- Blickwinkel: **0–45° gegen die Tischfläche**, 0° bedeutet horizontale Blickachse.
- Native Fisheye-Bilder mit 150–165° horizontalem Sichtfeld werden vor der Inferenz kalibriert entzerrt.
- Modelleingang: **320 × 192 RGB**, Float32 im Bereich 0–1.

| Datei | SHA-256 |
| --- | --- |
| [best.pt](best.pt) | `8b8935917e3f4a05f13de04311d21d953b9b55b6248663d7cc71941f9059a1a5` |
| [compact-stereo.onnx](compact-stereo.onnx) | `46e9f7dae1e30f844ff521a965692ae80a62120aaa9d895f5576a86fd83988a1` |

[Modellmanifest](../../configs/stereo/low-height-model-manifest.json) ·
[Inferenz mit mitgelieferten Rohbildbeispielen](../../docs/STEREO_HEIGHT_FISHEYE.md#inferenz-nach-einem-frischen-checkout) ·
[Messbericht](../../docs/stereo/height-fisheye/RESULTS.md)

Modell und implementierter Quellcode: [Apache-2.0](../../LICENSE).
Die echten Kameras benötigen ihre eigene Intrinsics-/Fisheye-Kalibrierung.
Flache Blickrichtungen und kleine Objektregionen können große Fehler erzeugen;
der Score ist keine kalibrierte Genauigkeitsgarantie.
