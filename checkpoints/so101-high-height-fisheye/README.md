# SO-101-Stereo: 42–60 cm, starke Fisheye-Optik

Dieses eigenständig trainierte CompactStereo-Modell ersetzt zusammen mit der
anderen Höhenvariante das frühere gemeinsame Stereo-Modell. Beide Varianten
wurden für 100 Epochen trainiert; der gewählte beste Checkpoint liegt hier bei
Epoche 100. Die Messungen stammen ausschließlich aus Isaac Sim.

- Senkrechte Sensorhöhe: **42–60 cm**; Kameraabstand **60 mm**.
- Relative Rotation: **0° in allen drei Achsen**.
- Blickwinkel: **0–45° gegen die Tischfläche**, 0° bedeutet horizontale Blickachse.
- Native Fisheye-Bilder mit 150–165° horizontalem Sichtfeld werden vor der Inferenz kalibriert entzerrt.
- Modelleingang: **480 × 288 RGB**, Float32 im Bereich 0–1.

| Datei | SHA-256 |
| --- | --- |
| [best.pt](best.pt) | `4d1a14e0fdb6f83804366bc5104b50314ceba2794919e9eefcd182d2b5977f88` |
| [compact-stereo.onnx](compact-stereo.onnx) | `5a7029de9822294c5a32360c9591b5de778b3888730fc5a15c0a018c688cd5f4` |

[Modellmanifest](../../configs/stereo/high-height-model-manifest.json) ·
[Inferenz mit mitgelieferten Rohbildbeispielen](../../docs/STEREO_HEIGHT_FISHEYE.md#inferenz-nach-einem-frischen-checkout) ·
[Messbericht](../../docs/stereo/height-fisheye/RESULTS.md)

Modell und implementierter Quellcode: [Apache-2.0](../../LICENSE).
Die echten Kameras benötigen ihre eigene Intrinsics-/Fisheye-Kalibrierung.
Flache Blickrichtungen und kleine Objektregionen können große Fehler erzeugen;
der Score ist keine kalibrierte Genauigkeitsgarantie.
