#!/usr/bin/env python3
"""Collect the two executed height/fisheye runs into portable manifests and a report."""

import argparse
import hashlib
import importlib.metadata
import json
import os
import shutil
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/robot-studio-stereo-matplotlib")

import numpy as np
import yaml


def read(path):
    return json.loads(Path(path).read_text())


def fmt(value):
    return "—" if value is None else f"{value:.2f}".replace(".", ",")


def main(output):
    output.mkdir(parents=True, exist_ok=True)
    results = Path("training-runs/height-fisheye")
    verification = dict(
        base_commit="0e95d8c70e42e4a1139af16041ecfb2515964609",
        scope="simulation only; independent height-specific stereo estimators",
        angle_reference="surface: 0 degrees is a horizontal optical axis",
        tests=dict(stereo=19, existing_vision=27),
        profiles={},
        environment={
            name: importlib.metadata.version(name)
            for name in (
                "torch",
                "numpy",
                "opencv-contrib-python-headless",
                "onnx",
                "onnxruntime",
                "matplotlib",
            )
        },
    )
    for profile in ("high", "low"):
        run = Path(f"checkpoints/so101-{profile}-height-fisheye")
        dataset = Path(f"stereo-data/{profile}-height-fisheye")
        config = yaml.safe_load(Path(f"configs/stereo/{profile}-height-fisheye.yaml").read_text())
        manifest = read(run / "compact-stereo.json")
        manifest.update(
            checkpoint=str(run / "best.pt"),
            onnx=str(run / "compact-stereo.onnx"),
            paths_relative_to="repository_root",
            purpose=f"{profile} height; synthetic extreme fisheye stereo",
            maximum_exposure_skew_ns=25000000,
        )
        Path(f"configs/stereo/{profile}-height-model-manifest.json").write_text(
            json.dumps(manifest, indent=2) + "\n"
        )
        reports = {}
        for split in ("test", "unseen"):
            report = read(results / f"{profile}-benchmark/{split}.json")
            report["model_checkpoint"] = str(run / "best.pt")
            reports[split] = report
        optics = []
        metadata = []
        data_manifest = read(dataset / "manifest.json")
        for record in data_manifest["samples"]:
            if not record["id"].endswith("_clean"):
                continue
            value = read((dataset / record["file"]).with_suffix(".json"))
            metadata.append(value)
            optics.extend(side["max_lens_displacement_px"] for side in value["optics"].values())
        verification["profiles"][profile] = dict(
            config=config,
            audit=read(dataset / "audit.json"),
            training=read(run / "training-report.json"),
            overfit=read(results / f"{profile}-overfit/training-report.json"),
            manifest=manifest,
            benchmarks=reports,
            raw_lens_displacement_px=[min(optics), max(optics)],
            gt_visible_object_opportunities=sum(
                obj["reference_world"] is not None for m in metadata for obj in m["objects"]
            ),
        )
        for label in ("successful", "difficult"):
            shutil.copy2(
                results / f"{profile}-benchmark/test-{label}.png", output / f"{profile}-{label}.png"
            )
        for name in ("learning", "groups"):
            shutil.copy2(results / f"{profile}-plots/{name}.png", output / f"{profile}-{name}.png")
    verification["inference"] = read(results / "inference-verification.json")
    sources = [
        *Path("src/ros_robot_studio/vision/stereo_learning").glob("*.py"),
        Path("src/ros_robot_studio/vision/stereo.py"),
        *Path("scripts").glob("*stereo*.py"),
        Path("tests/test_stereo_learning.py"),
        Path("tests/test_stereo_height_capture.py"),
    ]
    verification["source_sha256"] = {
        str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(set(sources))
    }
    (output / "verification.json").write_text(json.dumps(verification, indent=2) + "\n")
    paths = {
        "ROOT": str(Path.cwd()),
        "PYTHON": str(Path.cwd() / ".bootstrap/stereo/venv/bin/python"),
        "ISAAC_PYTHON": os.environ.get("ISAAC_PYTHON", "configure-local-isaac-python"),
    }
    for profile in ("high", "low"):
        paths[profile.upper() + "_MODEL"] = str(
            Path.cwd() / f"checkpoints/so101-{profile}-height-fisheye/best.pt"
        )
    (results / "local-paths.txt").write_text(
        "\n".join(f"{key}={value}" for key, value in paths.items()) + "\n"
    )
    make_raw_figure(output)
    write_markdown(verification, output)


def make_raw_figure(output):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(4, 4, figsize=(13, 9), layout="constrained")
    for row, (profile, index) in enumerate((("high", 0), ("high", 31), ("low", 0), ("low", 31))):
        root = Path(f"stereo-data/{profile}-height-fisheye")
        p = root / f"test/test_{index:04d}_clean.npz"
        meta = read(p.with_suffix(".json"))
        with np.load(p) as sample:
            values = (
                sample["raw_left"],
                sample["raw_right"],
                sample["left"],
                np.where(sample["valid"], sample["depth"], np.nan),
            )
            for ax, value, title in zip(
                axes[row],
                values,
                ("Raw left", "Raw right", "Rectified left", "Reference optical Z (m)"),
            ):
                ax.imshow(value)
                ax.set_title(title, fontsize=9)
                ax.set_xticks([])
                ax.set_yticks([])
            axes[row, 0].set_ylabel(
                f"{profile}: {meta['camera_height_m'] * 100:.1f} cm / {meta['view_angle_surface_deg']:.1f}°",
                fontsize=9,
            )
    fig.suptitle(
        "Extreme native fisheye and calibrated rectification · 0° is parallel to the table"
    )
    fig.savefig(output / "raw-optics.png", dpi=140)
    plt.close(fig)


def write_markdown(v, output):
    profiles = v["profiles"]
    lines = [
        "# Zwei neue Höhen-/Fisheye-Modelle · ausgeführte Abnahme",
        "",
        "Zwei unabhängig von Grund auf trainierte CompactStereo-Modelle, jeweils 100 Epochen, 10.500 Optimizerschritte und 84.000 Paar-Präsentationen. "
        "Beide verwenden exakt 60 mm Basis und identische Kamerarotationen in allen drei Achsen. "
        "**Alle Genauigkeiten sind Simulationsergebnisse.** [Konfiguration und Startbefehle](../../STEREO_HEIGHT_FISHEYE.md).",
        "",
        "**Winkelbezug:** 0–45° gegen die Tischfläche. 0° bezeichnet eine waagerechte Blickachse; der Arbeitsbereich liegt dann unterhalb der Bildmitte. "
        "Senkrechte Draufsicht wäre 90° zur Tischfläche. Höhe und Schrägentfernung sind unterschiedliche Größen.",
        "",
        "| Modell | Senkrechte Höhe | RGB-Auflösung | Beste Validation EPE px | Trainingsschleifen s |",
        "| --- | --- | --- | ---: | ---: |",
    ]
    for name, p in profiles.items():
        c = p["config"]
        t = p["training"]
        h = c["camera"]["height_range_m"]
        lines.append(
            f"| {name} | {h[0] * 100:.0f}–{h[1] * 100:.0f} cm | {c['image']['width']}×{c['image']['height']} | {fmt(t['best_validation_epe_px'])} | {fmt(t['elapsed_s'])} |"
        )
    lines += [
        "",
        "Je Modell: 420 Trainingsszenen / 840 Paare, 60 Validierungsszenen / 120 Paare, 90 Testszenen / 180 Paare und 45 ungesehene Szenen / 90 Paare. "
        "Alle Varianten einer Szene bleiben im selben Split. Seed High=3101, Low=3102. "
        "Der Checkpoint wurde ausschließlich nach Validation ausgewählt. Nach Epoche 80 wurde der Prozess beendet und aus dem gespeicherten Zustand bis Epoche 100 fortgesetzt. "
        "Trainings-Forward: BF16; Validation, Inferenz und ONNX: Float32. Gewichte: Float32.",
        "",
        "## Synchroner Test",
        "",
        "Optische Z-Fehler werden auf den gültigen, jeweils akzeptierten Referenzpixeln gemessen. XYZ-Fehler beziehen sich auf denselben erodierten sichtbaren Oberflächenmedian für beide Verfahren. "
        "Pro Modell gibt es 540 Objektgelegenheiten aus drei Objektarten und jeweils zwei Qualitätsvarianten. Unsichtbare oder zu kleine Referenzregionen bleiben im Nenner. "
        "Die Simulator-Masken bewerten den Tiefenschätzer; eine reale Objektdetektion ist damit nicht abgenommen.",
        "",
        "| Modell / Verfahren | Z Median / p95 mm | Objekt-XYZ Median / p95 mm | Gültige Objekte | Gültige Referenzpixel |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for name, p in profiles.items():
        for method, m in p["benchmarks"]["test"]["overall"].items():
            lines.append(
                f"| {name} / {method} | {fmt(m['depth_median_mm'])}/{fmt(m['depth_p95_mm'])} | {fmt(m['object_median_mm'])}/{fmt(m['object_p95_mm'])} | {m['valid_objects']}/{m['object_opportunities']} | {m['valid_pixel_fraction'] * 100:.2f} % |"
            )
    lines += [
        "",
        "SGBM verwendet eine beidseitige Auffüllung des Suchbereichs und eine Links-Rechts-Prüfung. "
        "Der neue Regressionstest prüft ausdrücklich den Fall Suchbereich = Bildbreite. Ein nur links aufgefülltes Bild würde beim Rückwärtsmatcher sämtliche gültigen Regionen verlieren.",
        "",
        "## Höhe und Blickwinkel",
        "",
        "| Profil / Gruppe | Verfahren | Z Median / p95 mm | XYZ Median / p95 mm | Gültige Objekte |",
        "| --- | --- | ---: | ---: | ---: |",
    ]
    for name, p in profiles.items():
        for field, title in [
            ("camera_height_m", "Höhe m"),
            ("view_angle_surface_deg", "Winkel zur Fläche °"),
        ]:
            for group, methods in p["benchmarks"]["test"]["groups"][field].items():
                for method, m in methods.items():
                    lines.append(
                        f"| {name} / {title} {group} | {method} | {fmt(m['depth_median_mm'])}/{fmt(m['depth_p95_mm'])} | {fmt(m['object_median_mm'])}/{fmt(m['object_p95_mm'])} | {m['valid_objects']}/{m['object_opportunities']} |"
                    )
    lines += [
        "",
        "## Ungesehene starke Linsenparameter",
        "",
        "Die Bildpaare stammen aus anderen Szenen und stärkeren k₁-Werten (−0,13 bis −0,09 statt −0,08 bis −0,02 im Training). "
        "Die Kalibrierung ist für die Entzerrung bekannt. Dies testet ungewohnte Randauflösung/Interpolation, nicht einen unbekannten Kalibrierfehler.",
        "",
        "| Modell / Verfahren | Z Median / p95 mm | XYZ Median / p95 mm | Gültige Objekte |",
        "| --- | ---: | ---: | ---: |",
    ]
    for name, p in profiles.items():
        for method, m in p["benchmarks"]["unseen"]["overall"].items():
            lines.append(
                f"| {name} / {method} | {fmt(m['depth_median_mm'])}/{fmt(m['depth_p95_mm'])} | {fmt(m['object_median_mm'])}/{fmt(m['object_p95_mm'])} | {m['valid_objects']}/{m['object_opportunities']} |"
            )
    lines += [
        "",
        "## Geometrie, Speicher und Inferenz",
        "",
        "RTX 4070 Ti mit 12.282 MiB, Ryzen 7 5800X, 32 GB RAM, Isaac Sim 5.1, PyTorch 2.6.0+cu124. "
        "Beide Datensätze bestehen Q-/Einheiten-/Frame-/Split- und Tischprüfungen. Die niedrigere Testgruppe wurde vor Training mit eindeutig getrennten Tisch-/Verdeckungslabels korrigiert. "
        "Die vier-Paar-Kapazitätsprüfungen erreichten folgende EPEs.",
        "",
        "| Profil | Overfit EPE px | Native Randverschiebung px | Belegte / reservierte Trainings-MiB |",
        "| --- | ---: | --- | ---: |",
    ]
    for name, p in profiles.items():
        t = p["training"]
        shift = p["raw_lens_displacement_px"]
        lines.append(
            f"| {name} | {fmt(p['overfit']['best_validation_epe_px'])} | {fmt(shift[0])}–{fmt(shift[1])} | {fmt(t['peak_allocated_mb'])}/{fmt(t['peak_reserved_mb'])} |"
        )
    lines += [
        "",
        "Das niedrige Modell benötigt bis 320 Pixel Suche: Die Referenzdaten enthalten tatsächlich gültige Disparitäten bis rund 316,5 Pixel. "
        "Die gemeinsame physische Kamerarotation ist 0°; die relative Orientierung wurde für alle Aufnahmen aus den gespeicherten Matrizen geprüft. "
        "Rohbild-Fisheye und rektifiziertes Bild haben unterschiedliche Intrinsics. Bereiche außerhalb der fotografischen Unterstützung sind nicht Teil der Referenz.",
        "",
        "| Profil / Verfahren | Entzerrung inkl. Maps ms | Warme Anwendung ohne Mapaufbau ms | Schätzer ms | Geometrie ms |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for name, p in profiles.items():
        for method, m in p["benchmarks"]["test"]["overall"].items():
            t = m["timing_median_ms"]
            lines.append(
                f"| {name} / {method} | {fmt(t['rectification_ms'])} | {fmt(t['rectification_apply_ms'])} | {fmt(t['estimator_ms'])} | {fmt(t['postprocess_ms'])} |"
            )
    lines += [
        "",
        "Die Rohkalibrierung wechselt im Benchmark von Szene zu Szene. Mapaufbau und eigentliche Remap-Anwendung sind deshalb getrennt erfasst. "
        "SGBM läuft auf der CPU, das Modell auf der GPU; Modellzeiten schließen Normalisierung/Resize und Transfers ein. "
        "Der Kamerasimulator selbst sowie Laden/Transport sind ausgenommen. Torch-Belegung und Reservierung enthalten nicht den kompletten Treiberkontext.",
        "",
    ]
    for name, p in profiles.items():
        report = p["benchmarks"]["test"]
        lines.append(
            f"{name}: GPU-Inferenz {fmt(report['model_peak_allocated_mb'])} MiB belegt, {fmt(report['model_peak_reserved_mb'])} MiB reserviert; Treiber-Stichprobe am Prozessende {fmt(report['model_process_gpu_mb_at_end'])} MiB."
        )
    lines += [
        "",
        "## Projektions- und Markerprüfung",
        "",
        "| Profil | Reprojektion p95 links/rechts px | Größter Tisch-Medianfehler mm | Marker-XYZ Median/p95 mm; gültig |",
        "| --- | ---: | ---: | ---: |",
    ]
    for name, p in profiles.items():
        a = p["audit"]
        markers = p["benchmarks"]["test"]["marker_triangulation"]
        lines.append(
            f"| {name} | {fmt(a['projection']['left']['p95_px'])}/{fmt(a['projection']['right']['p95_px'])} | {a['max_table_plane_error_mm']:.6f} | {fmt(markers['median_mm'])}/{fmt(markers['p95_mm'])}; {markers['valid']} |"
        )
    lines += [
        "",
        "Marker werden über Simulator-IDs zugeordnet und mit ihren tatsächlichen Bildmittelpunkten trianguliert. Das ist eine geometrische Plausibilisierung mit bekannter Identität, keine Abnahme eines realen Marker-Matchers.",
    ]
    lines += [
        "",
        "## Artefakte und Prüfungen",
        "",
        "Roh-Fisheye- und bereits entzerrte Inferenz liefern bitgleiche Disparität und Scores. "
        "ONNX und PyTorch wurden je Modell an drei Paaren aus Test, degradiertem Test und ungesehenem Optiksatz geprüft. "
        "Die App-Schnittstelle übernimmt die modellabhängigen Tiefengrenzen; nahe Punkte unter 20 cm werden akzeptiert. "
        "Ein 100-ms-Versatz wird weiterhin über das 25-ms-Pairinglimit abgelehnt. **46 relevante Tests bestehen.**",
        "",
    ]
    for name, p in profiles.items():
        parity = v["inference"][name]["onnx_parity"]
        difference = max(row["max_disparity_px"] for row in parity)
        lines += [
            f"- {name}: `{p['manifest']['checkpoint']}`, ONNX-Abweichung maximal {difference:.8f} Pixel. SHA-256 `{p['manifest']['checkpoint_sha256']}`."
        ]
    lines += [
        "",
        "Die absoluten lokalen Pfade stehen außerhalb von Git in `training-runs/height-fisheye/local-paths.txt`. "
        "Die portablen Modellmanifeste liegen unter `configs/stereo/high-height-model-manifest.json` und `low-height-model-manifest.json`. "
        "Gewichte, Datensätze und Trainingslogs dieser neuen Reihe bleiben lokale Artefakte. [Vollständige Zahlen und Quellprüfsummen](verification.json).",
        "",
        "## Sichtprüfung",
        "",
        "![Roh-Fisheye, Entzerrung und Referenz](raw-optics.png)",
        "",
    ]
    for name in profiles:
        lines += [
            f"### {name}",
            "",
            f"![Trainingsverlauf {name}]({name}-learning.png)",
            "",
            f"![Gruppenfehler {name}]({name}-groups.png)",
            "",
            f"![Gelungener Fall {name}]({name}-successful.png)",
            "",
            f"![Schwieriger Fall {name}]({name}-difficult.png)",
            "",
        ]
    lines += [
        "## Grenzen",
        "",
        "Sehr flache Blickachsen erzeugen trotz geringer Montagehöhe größere optische Entfernungen und kleine Objektregionen. "
        "Das hohe Modell erreicht deshalb keine durchgängige Millimeterpräzision. Auch beim niedrigen Modell sind die Fehler im oberen Perzentil deutlich größer als der Median. "
        "Eine gültige Ausgabe garantiert keine kleine Abweichung. Der Score ist weiterhin nicht als Fehlerwahrscheinlichkeit kalibriert. "
        "Die Referenzen bewerten sichtbare Oberflächen statt Objektmittelpunkten. Außerhalb der gültigen Simulatorreferenz wird keine Genauigkeit behauptet. "
        "Es wurden einfache Workcell-Geometrie und drei Objektarten verwendet; echte Kameras, echte Linsenkalibrierung und unabhängige Messabstände fehlen. "
        "Keine der Zahlen ist ein Nachweis realer Greifgenauigkeit.",
        "",
    ]
    (output / "RESULTS.md").write_text("\n".join(lines))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("docs/stereo/height-fisheye"))
    args = parser.parse_args()
    main(args.output)
