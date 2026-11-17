# Projects

In diesem Ordner kannst du Studio-Projekte (`.studio.yaml`) und Isaac-Sim-Szenen (`.usd`, `.usda`, `.usdc`) ablegen. Wähle in Studio **Project → Open…**, um ein Projekt mitsamt seinen Robotereinstellungen und Backendprofilen zu öffnen. Persönliche USB-Ports und Interpreterpfade werden lokal gespeichert.

- [SO-101-Real.studio.yaml](SO-101-Real.studio.yaml) verwendet das SO-101-Follower-Modell aus der [Offline-Vorlage](../examples/so101/so101.studio.yaml) mit USB-Einzelgelenksteuerung.
- Isaac-Szenen, die Studio für ein hier gespeichertes Projekt neu erzeugt, landen standardmäßig unter `isaac-scenes/`. Du kannst auch über **Simulation File → Browse…** eine vorhandene USD auswählen. Lass USD-Layer und referenzierte Dateien zusammen, damit relative Pfade gültig bleiben.

[USB einrichten und einzelne Gelenke bewegen](../README.md#physischen-so-101-follower-%C3%BCber-usb-anbinden)
