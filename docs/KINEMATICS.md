# Universal Kinematics & Pick-and-Place

## Bedienung

Die Erweiterung liegt in der vorhandenen **Robot Configuration**, rechts neben dem
unveränderten gemeinsamen 3D-Viewer. Es gibt zwei zusätzliche Untertabs:
**Kinematics & Motion** sowie **Positions & Tasks**. Keine ROS-Installation, kein
Docker und kein angeschlossenes Gerät sind erforderlich.

```bash
cd $HOME/Desktop/ROS_APP/ros-robot-studio
./start.sh --pick-place-demo
```

**Project → Open Pick-and-Place demo** öffnet dieselbe Funktion. Bei installiertem,
intaktem Originalmodell wird `SO101_PickAndPlace_Demo` aus der festen SO-101-Revision
aufgebaut und erneut validiert. Fehlt das Modell, bietet die GUI seine Installation
an; **No** öffnet den vollständigen generischen Lineararm. Die ursprüngliche
**Offline demo** mit Monitoring bleibt erhalten.

Optionale IK-Komponente: **Help → Setup assistant → Component verification →
Install Universal Kinematics Support**. Alternativ in der vorbereiteten lokalen Umgebung:

```bash
env -u PYTHONPATH .venv/bin/python -m pip install -r requirements-kinematics.lock.txt
```

SciPy 1.15.3 benötigt hier etwa 38 MB Download. Es gehört nicht zur Basiskomponente.
Der grafische Installer benutzt ausschließlich die aktuelle virtuelle Umgebung und
ändert mit `--no-deps` keine vorhandenen NumPy-/ROS-Pakete. Ohne SciPy starten Viewer,
Projektverwaltung und FK weiterhin; IK/Task-Wiedergabe werden mit Installationshinweis
deaktiviert. Nach Installation die Kette bestätigen und prüfen lassen.

## Kette bestätigen und kartesisch positionieren

1. Robotermodell auswählen/installieren oder eigene URDF importieren.
2. **Kinematics & Motion → Chain setup:** Base, Tool, Greiferzuordnung und aktive
   Arm-Joints prüfen. Arm- und Greifergelenke bleiben getrennt. SO-101-Metadaten
   ordnen `gripper` zu; bei generischen mehrdeutigen Endframes ist Bestätigung nötig.
   Die GUI bietet aktuell eine unabhängige Greiferachse an; die API unterstützt mehrere.
3. **Confirm chain & run FK / IK checks** führt Analyse und ein FK→IK→FK-Plausibilitätstest
   aus. **Robot Kinematics: Ready · offline only** gilt nur für diese Offline-Kette.
4. **Cartesian control:** expliziten Referenzframe und Tool prüfen, XYZ in mm und
   RPY in Grad eingeben. Intern werden ausschließlich Meter/Radiant verwendet.
5. **Calculate IK** läuft im Hintergrund. **Preview Pose** ändert die Viewerpose,
   **Apply to Offline Model** übernimmt sie ausdrücklich in die virtuelle Konfiguration.
   **Save Position** speichert das eingegebene kartesische Ziel mit editierbarem Namen.
   **Reset target / preview** stellt die aktuelle virtuelle Konfiguration wieder dar.

Position Only lässt die Orientierung frei und deaktiviert RPY. Axis erhält Position
und Richtung einer auswählbaren Toolachse X/Y/Z; Rotation um diese Achse bleibt frei.
Full Pose optimiert XYZ plus vollständige Orientierung. Es ist nur bei maximalem
numerisch beobachtetem Pose-Jacobi-Rang 6 auswählbar. Die Analyse betrachtet 13 feste
Konfigurationen, nicht bloß die Anzahl der Joints. Sie ist ein numerischer Fähigkeitstest,
kein mathematischer Beweis für jede Stellung. Lokale Singularitäten werden gesondert gemeldet.

Beim SO-101 sind fünf Armachsen und eine separate Greiferachse vorhanden. Der gemessene
maximale Pose-Rang ist 5, deshalb bleibt Full Pose deaktiviert. Position Only ist Standard.
Die Orientierung der bei Position Only gefundenen Lösung kann sich sichtbar ändern.

Alle Posen enthalten einen Referenzframe. Bekannte URDF-Frames werden über FK zum
gewählten Base-Frame transformiert. Unbekannte Kamera-/World-Frames ohne Transformation
werden abgewiesen. RPY ist URDF-konform `Rz(yaw) Ry(pitch) Rx(roll)`, Linktransformation
`T_parent · T_origin · T_joint`. Es gibt eine einzige FK für Solver, Viewer und Task-Compiler.
Meshes sind für Berechnung und Analyse nicht erforderlich.

## Teach-in und Task-Editor

**Teach-in positions** erfasst aktuelle Jointwerte oder die aktuelle Endeffektorpose.
Positionen lassen sich benennen, editieren, duplizieren, prüfen, aufrufen und löschen.
Referenzierte Positionen können erst nach Entfernen der Task-Verweise gelöscht werden.
Jointpositionen tragen eine SHA-256-Modellsignatur aus Geometrie, Jointnamen, Typen,
Ursprüngen, Achsen und Limits. Ein Modellwechsel erfordert explizites Neueinlernen.
Kartesische Ziele bleiben framedatengebunden und werden für jede neue Kette erneut gelöst.
Die GUI erlaubt dafür Änderungen an Tool- und Framenamen; sie ordnet nichts still um.

**Task editor** enthält sequentielle Schritte: Jointziel, kartesisches Ziel, Greifer
öffnen/schließen, Wait, Move Relative und Home. **Add/Edit/Remove/↑/↓** verändern
Schritte; **Compute selected step** berechnet den ausgewählten Schritt aus der aktuellen
virtuellen Pose. **Validate all** berechnet jeden Schritt in der tatsächlichen Reihenfolge.
Ein Fehler markiert den ersten fehlerhaften Schritt; kein Teilprogramm wird abgespielt.
**Preview**, **Pause/resume**, **Stop** und **Reset** steuern nur die Offline-Wiedergabe.
Stop hält die Pose, Reset stellt den Start der zuletzt berechneten Vorschau wieder her.

**Export/Import positions / tasks** speichert eine eigene YAML-Bibliothek (Schema 1).
Import validiert Daten, führt gleiche IDs zusammen und weist Namenskonflikte ab.
Robotersignaturen und Frames bleiben erhalten. Normales Projektspeichern sichert dieselben
Positionen, Tasks, Einstellungen und die aktuelle virtuelle Pose zusammen.

## Pick-and-Place-Assistent

Die sieben Seiten übernehmen den aktuellen Roboter; Pick und Place können aus Teach-in,
aktueller Pose, Joint-Eingabe oder kartesischer Eingabe stammen. Greiferwerte sind echte
URDF-Radiant/Metereinheiten. Die Schaltfläche **Suggest URDF extrema** liefert ausschließlich
virtuelle Vorschläge. Open/Close müssen ausdrücklich bestätigt werden; es gibt keine
Umrechnung in Hardware-Prozentwerte oder Motorkalibrierung.

Offsets für Pre-Pick/Pre-Place, Lift und Retreat haben jeweils Abstand, Richtung und
einen ausdrücklich gewählten Referenzframe. Die vorgeschlagenen Abstände hängen von
der Modellgröße ab und sind keine allgemein gültigen Anfahrwege. **Generate** erstellt
Home, Pre-Pick, Pick, Lift, Pre-Place, Place, Retreat und den vollständigen Task inklusive
Greifer/Dwell. Jeder Wegpunkt wird per IK und jede Verbindung dynamisch validiert.
Bei Fehlern können Offsets und Quellen korrigiert werden. Eingabeänderungen verwerfen
vorherige Validierungen, auch während einer laufenden Berechnung.

Auf Seite 7 zeigt **Preview in shared 3D viewer** den temporären Ablauf im gemeinsamen
Viewer. **Finish** übernimmt Positionen/Task in den Editor; Cancel behält sie nicht.
Der orange Würfel wird nach Close visuell an den Toolframe gebunden, folgt ihm und bleibt
nach Open am erreichten Ablageort. Das ist eine nicht physikalische Darstellung ohne
Kontakt-, Kraft- oder Stabilitätsmodell.

## Solverentscheidung und Grenzen

Verwendet wird **SciPy 1.15.3 `least_squares(method="trf")`** mit echten Bounds,
Jacobi-Näherung und skalierten Residuen. Der bereits getestete URDF-Parser und die FK
werden wiederverwendet. Vollständige Orientierungsfehler verwenden einen SO(3)-Rotationsvektor,
Axis-Fehler die Differenz der Tool-Richtungsvektoren. Positive Positions-/Orientierungsgewichte
sind über `IKOptions` konfigurierbar. Sie ersetzen keine getrennten Fehlertoleranzen.

Die konkrete Version unterstützt beschränkte Trust-Region-Optimierung und Python 3.10.
[Offizielle SciPy-1.15.3-Dokumentation](https://docs.scipy.org/doc/scipy-1.15.3/reference/generated/scipy.optimize.least_squares.html).
IKPy wurde bewertet: dessen dokumentierter `orientation_mode="all"` ist in der betrachteten
Schnittstelle als nicht unterstützt markiert; eine weitere URDF-/FK-Implementierung bringt
hier keinen Vorteil. [IKPy-Chain-Dokumentation](https://ikpy.readthedocs.io/en/latest/chain.html).
Pinocchio bietet eine leistungsfähige C++-basierte Grundlage, erhöht für dieses kleine
Python-MVP aber Installations-/Integrationsaufwand; es wurde weder installiert noch als
Solver implementiert. [Offizielles Pinocchio-Projekt](https://github.com/stack-of-tasks/pinocchio).

Die Suche startet bei der aktuellen Konfiguration und zusätzlichen reproduzierbaren,
begrenzten Zufallsstartwerten. Gültige Lösungen werden nach normiertem Gelenkabstand,
Kondition und Restfehler verglichen. Es gibt keine globale Vollständigkeitsgarantie.
Ein Abbruch durch Evaluationslimit, Zeitlimit oder lokale Minima ist **kein Beweis für
mathematische Nichterreichbarkeit**. Fehlerresiduen bleiben sichtbar. Erfolg erfordert
Solverkonvergenz, Einhaltung der Bounds sowie unabhängige FK-Prüfung innerhalb beider
Toleranzen. Ungültige Ergebnisse stellen keine anwendbaren Jointwerte bereit.

Unterstützt werden serielle offene Ketten in URDF-Bäumen: revolute, continuous, prismatic,
fixed und linear gekoppelte mimic-Joints. Mimic-Grenzen beschränken den unabhängigen Treiber.
Geschlossene Ketten, parallele Mechanismen, floating und planar sind für IK nicht unterstützt.
Continuous-Joints behalten unwrapped Zahlenwerte und werden auf dem kürzesten Winkelweg
interpoliert, soweit keine gekoppelten endlichen Grenzen dagegenstehen.

## Trajektorien, Sicherheit und Erweiterungen

Alle Bewegungen verwenden synchronisierte **Gelenkraumkurven** mit
`s(u)=3u²−2u³`. Bei einer Gelenkdifferenz Δq gelten die exakten Spitzen
`v_max=1.5|Δq|/T`, `a_max=6|Δq|/T²`. Die Segmentzeit wird daraus für alle Joints,
einschließlich mimic, bestimmt. URDF-Velocitylimits sowie vorhandene Beschleunigungsgrenzen
(URDF-Erweiterung `acceleration` oder projektspezifische Overrides) werden geprüft.
Fehlende Velocitylimits verwenden ausdrücklich Demo-Fallbacks .35 rad/s bzw. .05 m/s,
zusätzlich skaliert durch Preview Speed. Fehlende Beschleunigungslimits werden gemeldet.
Null-Limits erlauben keine Bewegung. Grenzwerte, Kontinuität und Samples werden geprüft.

Move Cartesian und Move Relative lösen **Endpunkte**. Eine geradlinige kartesische Bahn
ist nicht implementiert und im GUI nicht auswählbar. Ein Transformations-Gizmo fehlt.

**Collision checking not available – preview only** bleibt sichtbar. Keine Selbst-/Umgebungs-
Kollisionsprüfung, keine Hardwarefreigabe, keine Controller-/Kalibrierungs-/Not-Halt-Prüfung.
Ein IK-Erfolg und ein Software-Stopp ersetzen keine reale Bewegungsfreigabe bzw. physischen Not-Halt.

`KinematicsSolverManager` erlaubt zusätzliche Solver; MoveIt 2, KDL, TRAC-IK und Pinocchio
sind nur geplante Einträge. `CollisionValidator` ist ein Erweiterungsvertrag;
`CompiledTask.hardware_ready` ist stets false. `ExecutionBackend` trennt Tasklogik und
Ausführung; nur `OfflinePreviewBackend` ist implementiert. ROS 2/FollowJointTrajectory,
ros2_control und Isaac Sim sind nicht angebunden. Die internen SI-/Frame-/Taskdaten sind
von Humble/Jazzy/Lyrical unabhängig. Künftige ROS-Ausführung gehört in die vorhandene
Container-/Kommunikationsarchitektur, nicht als `rclpy`-Import in die GUI.

`PoseProvider` hat funktionierende Manual-/Saved-Implementierungen. TF-/Vision-Provider
melden ausdrücklich NotImplemented. Vision benötigt eine kalibrierte 3D-Lokalisierung
(z. B. Tiefe/bekannte Geometrie); 2D-Erkennung allein liefert keine vollständige 3D-Pose.
Dynamische Sensortrigger, Bedingungen, Schleifen und Verzweigungen sind noch nicht im
sequentiellen Interpreter implementiert; versionierte Schritte/IDs und Extensions erlauben
spätere explizite Schemaerweiterungen.
