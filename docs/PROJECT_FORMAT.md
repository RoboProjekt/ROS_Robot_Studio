# Projektformat v2

UTF-8-YAML, validiert durch `core.models.Project`. Unbekannte Felder und unbekannte
Formatversionen werden mit verständlichen Validierungsfehlern abgewiesen. YAML verwendet
`safe_load`; URDF-XML wird mit defusedxml gelesen. UUIDs identifizieren Projekte und Sensoren. v1-Dateien werden beim Laden im Speicher
auf v2 angehoben und erst beim bewussten Speichern als v2 geschrieben. Ältere Studio-
Versionen können v2-Projekte nicht lesen; vorhandene v1-Quelldateien bleiben beim Laden erhalten.

| Feld | Bedeutung |
| --- | --- |
| `format_version` | Neu: 2; Version 1 wird beim Laden migriert |
| `id`, `name`, `description` | Identität und Beschreibung |
| `default_ros` | Gespeicherte Präferenz: humble, jazzy oder lyrical |
| `robot` | Profil-ID, Modell-ID/-version/-revision, URDF, Gelenkpositionen, Endeffektor, Mapping, geplantes Sim-Backend sowie Hardware/Controller/IK |
| `sensors` | Typisierte Sensorquellen mit IDs, Topics, Einheiten, Rate, Grenzen, TF und Aktivstatus |
| `monitoring` | Topic, ausgewählte Felder, Zeitfenster 5–300 s, Renderfrequenz 1–30 Hz |
| `runtime` | Loopback-Port, Speicher-/CPU-Limits und getrennte distributionsspezifische Overrides |

Ein vollständiges direkt ladbares Beispiel liegt unter `examples/generic_robot/demo.studio.yaml`.

```yaml
format_version: 2
id: research-arm-01
name: Research Arm
description: Laboratory configuration
default_ros: humble
robot:
  profile: generic-arm
  model_id: null
  model_version: null
  source_revision: null
  joint_positions: {}
  joint_mapping_status: not_calibrated
  urdf: research.assets/robot.urdf
  source_xacro: null
  hardware: null
  controller: null
  ik_solver: null
  end_effector: tool_link
sensors:
  - id: wrist-sensor-01
    name: Wrist force
    kind: Force / torque
    driver: ros-topic
    topic: /wrist/force
    parent_frame: tool_link
    transform:
      kind: static
      translation: [0.0, 0.0, 0.04]
      rotation: [0.0, 0.0, 0.0]
    quantity: force
    unit: N
    sample_rate: 20.0
    lower_limit: -10.0
    upper_limit: 10.0
    warning: 8.0
    enabled: true
monitoring:
  topic: /studio/sine
  fields: [data]
  window_seconds: 30
  update_hz: 20
runtime:
  port: 9090
  memory_mb: 2048
  cpu_count: 2.0
  overrides: {}
```

Translation ist in Metern, RPY in Radiant. `static`/`dynamic` beschreibt die Absicht;
es wird in diesem MVP noch keine Transformation als ROS-TF veröffentlicht. Sensor-
Abtastraten und Grenzwerte sind Konfiguration, keine Treiberbefehle oder Sicherheitssteuerung.

Relative URDF-Pfade beziehen sich auf die YAML-Datei. Beim Speichern/Exportieren wird
eine URDF-Kopie nach `<Projektdateistamm>.assets/robot.urdf` geschrieben. Beide zusammen
verschieben. Relative STL-Meshes innerhalb der URDF-Wurzel werden ebenfalls kopiert.
Package-URIs benötigen eine explizite Paketzuordnung im Loader-API und sind im GUI-Export
noch nicht automatisch portierbar. Eine
expandierte Xacro-Datei wird als URDF gespeichert; `source_xacro` bleibt Herkunftsinformation.

Das Sitzungs-Dropdown ist nicht `default_ros`. Der Nutzer übernimmt einen neuen
Projektstandard ausdrücklich. Konfigurationen für Zieldistributionen gehören künftig in
separate erzeugte Verzeichnisse bzw. `runtime.overrides`; dieser MVP führt keine
Quellcodemigration oder Änderung ursprünglicher ROS-Konfigurationen durch.

User-Einstellungen liegen unter `$XDG_CONFIG_HOME/ros-robot-studio/settings.json`,
Logs unter `$XDG_STATE_HOME/ros-robot-studio/logs/`, expandierte Xacros unter
`$XDG_CACHE_HOME/ros-robot-studio/expanded/`. Ohne gesetzte Variablen gelten die
üblichen Linux-XDG-Standardpfade.


## Installierte Katalogmodelle

Für Katalogmodelle enthält `robot.urdf` statt eines lokalen Benutzerpfads eine
`model://<model_id>/<source_revision>/<urdf>`-Referenz. `model_id`, `model_version` und
`source_revision` müssen dazu passen. Beim Laden wird genau diese Revision geprüft;
fehlende oder beschädigte Modelldateien werden nach ausdrücklicher Bestätigung repariert.
Keine andere Version wird still substituiert, die bisherige Konfiguration bleibt bis zum
erfolgreichen Laden erhalten. Exportieren kopiert die Referenz, keine großen Meshes.

`robot.joint_positions` enthält ausschließlich unabhängig bewegliche URDF-Joints in
SI-Einheiten. Feste Joints werden nicht gesetzt, mimic-Joints ergeben sich aus der URDF.
Unbekannte Namen, nicht endliche Werte und Grenzverletzungen werden beim Modellladen
abgewiesen. `end_effector` bezeichnet einen tatsächlich vorhandenen Link-Frame.

Die vorbereitete `simulation`-Struktur enthält ausschließlich Isaac-Sim-Planung:
Backend `isaac_sim`, Status `planned`, Assetformat `usd`, Pfad null und Joint-Mapping
`not_configured`. `joint_mapping_status: not_calibrated` behauptet keine Hardwareumrechnung.
Vollständiges Beispiel: `examples/so101/so101.studio.yaml`; weitere Details: [SO101.md](SO101.md).

Originalmodell-Dateien liegen unter `$XDG_DATA_HOME/ros-robot-studio/robots/` (Standard:
`~/.local/share/ros-robot-studio/robots/`). Das ist persistente Modellablage, kein
flüchtiger Buildcache. `installation.json` hält Quellrevision, Dateihashes und Zeitpunkt fest.
