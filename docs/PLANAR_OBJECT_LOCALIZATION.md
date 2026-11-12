# Planare Objektlokalisierung

Studio bildet aus einem ausgewählten Pixel `(u, v)` mithilfe der Kamera-Intrinsics und der Verzerrungskorrektur einen Strahl im optischen Frame. TF2 transformiert Ursprung und Richtung zum Aufnahmezeitpunkt in den Ebenenframe. Für Ebenenpunkt `p`, Einheitsnormale `n`, Strahlursprung `o` und Richtung `d` gilt:

```text
t = dot(n, p − o) / dot(n, d)
position = o + t · d
```

Parallele Strahlen, Schnitte hinter der Kamera und Punkte außerhalb eines konfigurierten Arbeitsbereichs werden abgewiesen. Studio kann den gültigen Schnittpunkt in Welt-, optischem oder Roboterbasisframe anzeigen. Für jeden Ausgabeframe muss zur Aufnahmezeit eine gültige Transformationskette verfügbar sein. Bei fehlenden Daten bleibt das 2D-Ergebnis verfügbar.

Als Referenz stehen Bounding-Box-Mitte, Mitte der Unterkante und ein manuell gewählter Pixel zur Verfügung. Die Wahl des Referenzpixels beeinflusst die Abweichung zum tatsächlichen Kontaktpunkt. Bei einem Objekt mit Höhe kann beispielsweise die Oberkante von oben sichtbar sein, während sein Kontaktpunkt verdeckt liegt. Die optionale Homographie-Kalibrierung ist ein späterer Entwicklungsschritt.

Der `VisionPoseProvider` liefert den gerahmten planaren Punkt für ein gültiges aktuelles Ergebnis. Über **Use as Target Draft…** lässt sich ein Entwurf im Kinematikeditor öffnen. Dafür müssen Basisframe, Werkzeugorientierung und XYZ-Versatz ausdrücklich angegeben werden. Ein Entwurf löst weder IK-Ausführung noch Bewegung aus.

[Messabweichungen der SO-101-Demo](VISION_VERIFICATION.md)
