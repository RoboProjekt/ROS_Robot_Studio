# Zusätzliches Qt-Widget: Ergebnis

Das zusätzliche Objekt ist **kein verlorener Dateidialog und kein Studio-Panel**.
Es handelt sich um das interne Desktop-Hilfswidget von QApplication in Qt 6.9.3.

| Eigenschaft | Beobachtung |
|---|---|
| Python-/Qt-Klasse | `QWidget` / `QWidget` |
| Parent | keiner |
| Objektname / Fenstername | jeweils leer |
| Sichtbarkeit / Kinder | unsichtbar / keine Kinder |
| Typ | `Qt::Desktop` |
| Geometrie beim Auftreten | `(0, 0, 640, 480)` |
| Erstellung | native Cursor-Leave-Verarbeitung während der Taskdialog-Serie, vor der Dateiauswahl |
| Nach Events / `deleteLater()` / 1 s / 6 s | dasselbe Objekt bleibt vorhanden |
| Nativer Erstellungsaufruf | `QWidget::QWidget` ← `QApplicationPrivate::desktop()` |

Die Untersuchung verwendete `QApplication.allWidgets()`, QObject-Parent/Kinder,
Objekt-/Fensternamen, Metaobjektklasse, Sichtbarkeit, Widgettyp, C++-Objektidentität,
Qt-Eventfilter und einen gdb-Konstruktor-Breakpoint ausschließlich im eigenen Testprozess.
Ein isolierter QFileDialog allein hinterließ kein Widget. Ein weiterer Test ordnete die
Erstellung zunächst der Taskdialog-Serie und anschließend dem Qt-C++-Aufruf zu.
[Messdaten und ausgewählter nativer Stack](qt-widget-investigation.json).

Der [Upstreamcode der tatsächlich eingesetzten Version](https://github.com/qt/qtbase/blob/v6.9.3/src/widgets/kernel/qapplication.cpp#L2453)
erzeugt dieses Objekt bei Bedarf mit `new QWidget(nullptr, Qt::Desktop)` und hält es
anwendungsweit. Der [QApplication-Destruktor](https://github.com/qt/qtbase/blob/v6.9.3/src/widgets/kernel/qapplication.cpp#L690)
löscht es. Dass es das Schließen eines Studio-Fensters überlebt, entspricht seiner Ownership.

Es wurde **kein Produktionscode zum Löschen Qt-interner Objekte** ergänzt. Die native
Regression zählt dieses präzise identifizierte Hilfsobjekt separat: höchstens eine Instanz,
unsichtbar, ohne Parent/Kinder und mit derselben C++-Identität über alle Fensterzyklen.
Jedes andere zusätzliche Widget bleibt ein fehlgeschlagener Test. Für Studio-Panels,
Dialoge, Overlays, PlotWidgets, Timer und Worker gelten weiterhin die vollständigen
Freigabeprüfungen. Die früher verwendete absolute Nullzählung ohne Beachtung dieses
Qt-Anwendungsobjekts war als Leckkriterium zu grob.
