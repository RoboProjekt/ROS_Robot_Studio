#!/usr/bin/env python3
"""Plot saved training history, test subgroups and matched temporal robustness results."""

import argparse
import json
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/robot-studio-stereo-matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def plot(run, benchmark, output):
    output.mkdir(parents=True, exist_ok=True)
    history = json.loads((run / "history.json").read_text())
    epochs = [row["epoch"] + 1 for row in history]
    training_report = json.loads((run / "training-report.json").read_text())
    previous_elapsed = training_report.get("elapsed_s", 0) - training_report.get(
        "session_elapsed_s", training_report.get("elapsed_s", 0)
    )
    resume_epoch = (
        min(history, key=lambda row: abs(row["elapsed_s"] - previous_elapsed))["epoch"] + 1
        if previous_elapsed > 0.1
        else None
    )
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.7), layout="constrained")
    axes[0].plot(epochs, [r["loss"] for r in history], label="Training, weighted Smooth-L1")
    axes[1].plot(epochs, [r["validation_epe_px"] for r in history], label="All valid pixels")
    axes[1].plot(
        epochs, [r["validation_foreground_epe_px"] for r in history], label="Object pixels"
    )
    for ax in axes:
        ax.set_xlabel("Epoch")
        ax.grid(alpha=0.25)
        ax.legend(fontsize=8)
        if resume_epoch is not None:
            ax.axvline(resume_epoch, color="gray", linestyle="--", alpha=0.6)
    axes[0].set_ylabel("Loss")
    axes[1].set_ylabel("Validation disparity error (pixels)")
    fig.suptitle(
        f"SO-101 synthetic stereo · resumed after epoch {resume_epoch}"
        if resume_epoch is not None
        else "SO-101 synthetic stereo · training"
    )
    fig.savefig(output / "learning.png", dpi=150)
    plt.close(fig)

    test = json.loads((benchmark / "test.json").read_text())
    fig, axes = plt.subplots(1, 4, figsize=(13, 3.6), layout="constrained")
    height_run = "camera_height_m" in test["groups"]
    for ax, field, title in zip(
        axes,
        ["camera_height_m", "view_angle_surface_deg", "quality", "occluded"]
        if height_run
        else ["distance_m", "baseline_m", "blur_sigma", "occluded"],
        ["Vertical height (m)", "Angle to surface (deg)", "Raw image quality", "Occluder present"]
        if height_run
        else ["Working distance (m)", "Baseline (m)", "Blur sigma (px)", "Occluder present"],
    ):
        groups = test["groups"][field]
        labels = list(groups)
        x = np.arange(len(labels))
        for name, color in [("sgbm", "#ce7e2b"), ("learned", "#177ca5")]:
            ax.plot(
                x,
                [groups[k][name]["object_median_mm"] for k in labels],
                "o-",
                color=color,
                label=name,
            )
        ax.set_xticks(x, labels)
        ax.set_xlabel(title)
        ax.set_ylim(bottom=0)
        ax.grid(alpha=0.25)
    axes[0].set_ylabel("Median valid object XYZ error (mm)")
    axes[-1].legend()
    fig.suptitle("Held-out synchronous scenes · object coverage reported separately in RESULTS.md")
    fig.savefig(output / "groups.png", dpi=150)
    plt.close(fig)

    if not (benchmark / "asynchronous.json").exists():
        return
    temporal = json.loads((benchmark / "asynchronous.json").read_text())["groups"]["time_offset_ms"]
    labels = list(temporal)
    x = np.array([int(label) for label in labels])
    fig, axes = plt.subplots(1, 2, figsize=(9, 3.5), layout="constrained")
    for name, color in [("sgbm", "#ce7e2b"), ("learned", "#177ca5")]:
        axes[0].plot(
            x,
            [temporal[k][name]["object_median_mm"] for k in labels],
            "o-",
            color=color,
            label=name,
        )
        axes[0].plot(
            x,
            [temporal[k][name]["object_p95_mm"] for k in labels],
            "--",
            color=color,
            alpha=0.65,
            label=name + " p95",
        )
        axes[1].plot(
            x,
            [temporal[k][name]["object_valid_fraction"] * 100 for k in labels],
            "o-",
            color=color,
            label=name,
        )
    for ax in axes:
        ax.set_xlabel("Right exposure delay (ms)")
        ax.set_xticks(x)
        ax.grid(alpha=0.25)
        ax.legend(fontsize=8)
    axes[0].set_ylabel("Valid object XYZ error (mm)")
    axes[1].set_ylabel("Accepted object positions (%)")
    axes[1].set_ylim(0, 105)
    fig.suptitle("Same 36 scenes at every delay · two moving cubes · 0.3 m/s")
    fig.savefig(output / "temporal.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--benchmark", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    plot(args.run, args.benchmark, args.output)
