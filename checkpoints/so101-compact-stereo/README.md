# Geprüfter SO-101-Stereo-Modellstand

Dieser Ordner enthält den tatsächlich trainierten CompactStereo-Schätzer mit
11.210 Parametern. Er schätzt linke Disparität aus einem rektifizierten RGB-Paar.
Er wurde ausschließlich auf synthetischen Isaac-Sim-Daten geprüft.

| Datei | Inhalt | SHA-256 |
| --- | --- | --- |
| [best.pt](best.pt) | Bester Validierungscheckpoint, Epoche 76; für den Python-Schätzer | `792fa8af6fac4ca6b5a85f5867d4e1c5ca321697ed7faccb750b50f6b3423709` |
| [compact-stereo.onnx](compact-stereo.onnx) | Inferenzgraph, Opset 17, fester Eingang 1×3×192×320 | `dbd66fbbb75fcdf72232cc2219ab90b2db6fe354a9ad98d193002ad46cd7e086` |

Ein- und Ausgang, Normalisierung, Rückskalierung und Geometrie stehen im
[portablen Modellmanifest](../../configs/stereo/model-manifest.json).
Das Modell und sein hier implementierter Quellcode stehen unter
[Apache-2.0](../../LICENSE); Abhängigkeiten behalten ihre eigenen Lizenzen.

Die [Trainingsdokumentation](../../docs/STEREO_DEPTH_TRAINING.md) enthält
Installation und einen ausführbaren Inferenzbefehl für das mitgelieferte
synthetische RGB-Paar. [Messbericht](../../docs/stereo/RESULTS.md) und
[Kamera-Robustheitstests](../../docs/stereo/robustness-mild/RESULTS.md)
dokumentieren die Grenzen, insbesondere Zeitversatz und Kalibrierabweichungen.

Der umfangreiche Datensatz und lokale Trainingsausgaben sind nicht enthalten.
`last.pt` zur Fortsetzung des letzten lokalen Trainings bleibt ein lokales Artefakt.
