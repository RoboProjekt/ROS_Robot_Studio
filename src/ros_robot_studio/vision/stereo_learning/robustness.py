"""Run matched, mild camera-quality tests without changing data or training weights."""

import hashlib
import importlib.metadata
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml

from .camera_effects import RobustnessSuite
from .evaluation import benchmark


def run(config, dataset, checkpoint, output, effects_config, device=None):
    output = Path(output)
    if (output / "robustness.json").exists():
        raise ValueError("Choose a new output directory to preserve an existing robustness run")
    output.mkdir(parents=True, exist_ok=True)
    effects = yaml.safe_load(Path(effects_config).read_text())
    if effects["source_split"] not in ("validation", "test", "unseen", "asynchronous"):
        raise ValueError("Robustness evaluation requires a held-out split")
    if not effects.get("clean_only"):
        raise ValueError("Use clean source pairs to avoid confounding pre-existing blur/JPEG")
    suite = RobustnessSuite(effects)
    checksum = hashlib.sha256(Path(checkpoint).read_bytes()).hexdigest()
    reports, geometry = {}, {}
    for name, effect in suite.effects.items():
        print(f"CAMERA_PROFILE {name}", flush=True)

        def prepare(sample, metadata):
            transformed, statistics, _ = suite.transform(name, sample, metadata)
            return transformed, statistics

        report = benchmark(
            config,
            dataset,
            checkpoint,
            output / name,
            split=effects["source_split"],
            device=device,
            preprocess=prepare,
            clean_only=True,
        )
        report["model_checkpoint"] = Path(checkpoint).name
        reports[name] = report
        geometry[name] = dict(
            max_raw_displacement_px=max(float(m["shift_px"].max()) for m in effect.cache.values()),
            max_true_projection_roundtrip_px=max(
                float(m["roundtrip_px"].max()) for m in effect.cache.values()
            ),
            max_left_residual_px=max(
                float(m["residual_px"].max()) for key, m in effect.cache.items() if key[2] == "left"
            ),
            max_right_residual_px=max(
                float(m["residual_px"].max())
                for key, m in effect.cache.items()
                if key[2] == "right"
            ),
        )
        if geometry[name]["max_left_residual_px"] > 1e-5:
            raise ValueError("Left reference grid was not restored accurately")
    assert len({r["overall"]["learned"]["reference_pixels"] for r in reports.values()}) == 1
    assert checksum == hashlib.sha256(Path(checkpoint).read_bytes()).hexdigest()
    result = dict(
        schema_version=1,
        timestamp_utc=datetime.now(timezone.utc).isoformat(),
        environment={
            name: importlib.metadata.version(name)
            for name in ("numpy", "torch", "opencv-contrib-python-headless")
        },
        source_sha256={
            path.name: hashlib.sha256(path.read_bytes()).hexdigest()
            for path in Path(__file__).parent.glob("*.py")
        },
        effects_config=effects,
        checkpoint_sha256=checksum,
        source_manifest_sha256=hashlib.sha256(
            (Path(dataset) / "manifest.json").read_bytes()
        ).hexdigest(),
        source="clean held-out pairs from the existing Isaac Sim dataset",
        model_updated=False,
        reference="unchanged ideal left optical-Z, Q and object points",
        support="intersection of photographic support over all configured profiles",
        geometry=geometry,
        conditions=reports,
    )
    (output / "robustness.json").write_text(json.dumps(result, indent=2))
    make_figures(result, suite, dataset, output)
    write_report(result, output)
    return dict(
        output=str(output.resolve()),
        profiles=len(reports),
        checkpoint_sha256=checksum,
        samples_per_profile=next(iter(reports.values()))["overall"]["learned"]["samples"],
    )


def make_figures(result, suite, dataset, output):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    names = list(result["conditions"])
    x = np.arange(len(names))
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8), layout="constrained")
    for method, color, offset in (("sgbm", "#ce7e2b", -0.18), ("learned", "#177ca5", 0.18)):
        rows = [r["overall"][method] for r in result["conditions"].values()]
        for ax, metric in zip(axes, ("depth_p95_mm", "object_median_mm", "object_valid_fraction")):
            ax.bar(
                x + offset,
                [r[metric] * (100 if metric.endswith("fraction") else 1) for r in rows],
                width=0.34,
                label=method,
                color=color,
            )
    for ax, title in zip(
        axes, ("Depth p95 (mm)", "Object XYZ median (mm)", "Accepted objects (%)")
    ):
        ax.set_title(title)
        ax.set_xticks(x, [n.replace("_", "\n") for n in names], fontsize=7, rotation=25, ha="right")
        ax.grid(axis="y", alpha=0.2)
    axes[0].legend()
    axes[2].set_ylim(0, 105)
    fig.suptitle("Mild camera defects · identical held-out scenes and reference footprint")
    fig.savefig(output / "metrics.png", dpi=145)
    plt.close(fig)

    manifest = json.loads((Path(dataset) / "manifest.json").read_text())
    records = [
        r
        for r in manifest["samples"]
        if r["split"] == result["effects_config"]["source_split"] and r["id"].endswith("_clean")
    ]
    # Fixed scene, not selected according to error. Keep identical content in every row.
    record = records[min(56, len(records) - 1)]
    path = Path(dataset) / record["file"]
    metadata = json.loads(path.with_suffix(".json").read_text())
    with np.load(path) as archive:
        sample = {key: archive[key].copy() for key in archive.files}
    selected = [
        name
        for name in ("control", "blur_unequal", "fisheye_calibrated", "combined_mild")
        if name in names
    ]
    if not selected:
        selected = names[:4]
    fig, axes = plt.subplots(
        len(selected), 4, figsize=(13, 2.6 * len(selected)), layout="constrained", squeeze=False
    )
    for row, name in zip(axes, selected):
        transformed, _, raw = suite.transform(name, sample, metadata)
        for ax, image, title in zip(
            row,
            (sample["left"], raw["left"], raw["right"], transformed["left"]),
            ("Original left", "Raw left", "Raw right", "Recovered left"),
        ):
            ax.imshow(image)
            ax.set_title(title, fontsize=9)
            ax.set_xticks([])
            ax.set_yticks([])
        row[0].set_ylabel(name.replace("_", " "), fontsize=9)
    displacement = max(row["max_raw_displacement_px"] for row in result["geometry"].values())
    fig.suptitle(
        f"{record['id']} · camera effects at {sample['left'].shape[1]}×{sample['left'].shape[0]} · max lens shift {displacement:.2f} px"
    )
    fig.savefig(output / "camera-effects.png", dpi=145)
    plt.close(fig)


def write_report(result, output):
    reports = result["conditions"]
    first = next(iter(reports.values()))["overall"]["learned"]
    lines = [
        "# Milde Kameraabweichungen im Stereo-Test",
        "",
        f"Bestehender Modellstand, **{first['samples']} saubere Bildpaare je Profil**, "
        f"{len(reports)} Profile, {first['object_opportunities']} Objektgelegenheiten pro Profil. "
        "Nur zurückgehaltene Isaac-Szenen; Gewichte und Trainingsdaten wurden nicht verändert.",
        "",
        "Die Bedingungen verwenden exakt dieselben Quellbilder, Posen, ideale Ausgangskalibrierung und Objektreferenzpunkte. "
        "Eine gemeinsame fotografisch verfügbare Bildregion wird aus sämtlichen Profilen bestimmt. "
        "Alle Tiefen- und Objektmetriken verwenden diese gemeinsame Region. Das verhindert einen "
        "scheinbaren Genauigkeitsgewinn durch unterschiedlich beschnittene Bildränder. "
        "Die Zahlen sind deshalb nicht unmittelbar mit dem älteren Benchmark auf sauber/degradiert gemischten Bildern vergleichbar.",
        "",
        "| Profil | SGBM Z Median/p95 mm | Modell Z Median/p95 mm | SGBM XYZ Median/p95 mm; gültig | Modell XYZ Median/p95 mm; gültig |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for name, report in reports.items():
        a, b = (report["overall"][method] for method in ("sgbm", "learned"))
        lines.append(
            f"| {name} | {a['depth_median_mm']:.2f}/{a['depth_p95_mm']:.2f} | {b['depth_median_mm']:.2f}/{b['depth_p95_mm']:.2f} | "
            f"{a['object_median_mm']:.2f}/{a['object_p95_mm']:.2f}; {a['valid_objects']}/{a['object_opportunities']} | "
            f"{b['object_median_mm']:.2f}/{b['object_p95_mm']:.2f}; {b['valid_objects']}/{b['object_opportunities']} |"
        )
    lines += [
        "",
        "![Fehler und Objektabdeckung](metrics.png)",
        "",
        "## Tatsächlich angewandte Stärken",
        "",
        "Die Standardkonfiguration verwendet Blur mit Gauß-σ 0,4/0,8 Pixel, asymmetrisch 0,35/0,9 Pixel. Bewegungsunschärfe: horizontaler "
        "3-Pixel-Kernel. Linsen: Brown-Conrady-Tonnenverzeichnung und separat OpenCVs echtes Fisheye-θ-Modell. "
        "Die Fisheye-Koeffizienten kompensieren einen Großteil des Unterschieds zur Pinhole-Projektion "
        "bei diesem Sichtfeld. Deshalb ist die geometrische Verschiebung klein. Nullkoeffizienten "
        "würden im Fisheye-Modell dagegen keine Pinhole-Identität ergeben.",
        "",
        "| Profil | Größte Rohbildverschiebung px | Größter geometrischer Restfehler rechts px |",
        "| --- | ---: | ---: |",
    ]
    for name, values in result["geometry"].items():
        lines.append(
            f"| {name} | {values['max_raw_displacement_px']:.3f} | {values['max_right_residual_px']:.3f} |"
        )
    lines += [
        "",
        "Die Standard-Kalibrierfehlervariante verändert nur rechts k₁ von tatsächlich 0,300 auf angenommen "
        "0,285. Die linke Kamera wird geometrisch korrekt entzerrt. Ihre ursprüngliche Tiefenreferenz "
        "und Q bleiben dadurch gültig. Die rechte Kamera ist dann nur näherungsweise rektifiziert: "
        "genau diese kleine Fehlanpassung wird getestet. Beidseitige unbekannte Kalibrierfehler wurden hier nicht simuliert.",
        "",
        "Im Standard-Kombinationsprofil kommen σ=0,4/0,65 Pixel, Rauschen σ=1/255 und 3 % weniger Helligkeit rechts hinzu. "
        "Dies ist ein gemeinsamer Stressfall und isoliert die Einzelursachen nicht. Alle Parameter "
        "stehen in `configs/stereo/robustness-mild.yaml`. Die Stärken gelten bei 320×192 Pixeln.",
        "",
        "![Gleiche Szene unter milden Rohkameraeffekten](camera-effects.png)",
        "",
        "Die Rohbilder werden zuerst mit dem jeweiligen Linsenmodell abgebildet, dann unscharf/verrauscht "
        "und anschließend mit der bekannten bzw. leicht fehlerhaften Kalibrierung entzerrt. Ein bloßes "
        "Verbiegen rektifizierter RGB-Bilder bei unveränderten falschen Tiefenlabels wird vermieden. "
        "Numerische Tests prüfen Projektionsinversion, erhaltene Disparität einer bekannten Ebene, "
        "gemeinsame Referenzunterstützung und reproduzierbare Störungen.",
        "",
        "Kalibrierungskarten werden zwischengespeichert. Die Rektifizierungszeit im JSON enthält "
        "die tatsächliche Linsenkorrektur sowie die bestehende Stereo-Remap-Stufe. Das synthetische "
        "Erzeugen der Rohkameradefekte wird nicht als Inferenzzeit gezählt. Details einschließlich "
        "Abdeckung, Ablehnungsgründen und getrennten Laufzeiten stehen in `robustness.json` und in "
        "den pro Profil gespeicherten Berichten.",
        "",
        "Dies erweitert die Offline-Tests. Es ist kein zusätzliches Fisheye-Live-Backend in der GUI "
        "und kein auf diesen Testdaten nachtrainiertes Modell. Die angenommenen Linsenparameter "
        "sind synthetisch und keine Messung der AMDIS01B. Reale Kalibrierung und unabhängige Realtests bleiben erforderlich.",
        "",
        "Primärquellen: [OpenCV Fisheye-Modell](https://docs.opencv.org/4.x/db/d58/group__calib3d__fisheye.html), "
        "[OpenCV Pinhole-/Verzeichnungsmodell](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html).",
        "",
    ]
    parameters = ["", "## Parameter dieses Laufs", "", "| Profil | Parameter |", "| --- | --- |"]
    for profile in result["effects_config"]["profiles"]:
        values = {key: value for key, value in profile.items() if key != "id"}
        parameters.append(f"| {profile['id']} | `{json.dumps(values)}` |")
    (output / "RESULTS.md").write_text("\n".join(lines + parameters) + "\n")
