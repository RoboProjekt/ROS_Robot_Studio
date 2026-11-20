"""Deterministic supervised training with optimizer and RNG state for exact continuation."""

import hashlib
import json
import os
import random
import time
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader, Dataset

from .model import CompactStereo


class StereoDataset(Dataset):
    def __init__(self, root, split, limit=None):
        self.root = Path(root)
        self.manifest = json.loads((self.root / "manifest.json").read_text())
        self.records = [record for record in self.manifest["samples"] if record["split"] == split]
        if limit:
            self.records = self.records[:limit]
        if not self.records:
            raise ValueError(f"No samples for {split}")
        self.samples = []
        integrity_path = self.root / "data-integrity.json"
        integrity = (
            {r["file"]: r for r in json.loads(integrity_path.read_text())}
            if integrity_path.exists()
            else {}
        )
        for record in self.records:
            if integrity:
                actual = hashlib.sha256((self.root / record["file"]).read_bytes()).hexdigest()
                if actual != integrity[record["file"]]["sha256"]:
                    raise ValueError(f"Data checksum changed: {record['file']}")
            with np.load(self.root / record["file"]) as sample:
                self.samples.append(
                    {
                        key: sample[key].copy()
                        for key in ("left", "right", "disparity", "valid", "segmentation")
                    }
                )

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, index):
        sample = self.samples[index]
        return (
            torch.from_numpy(sample["left"].transpose(2, 0, 1).copy()).float() / 255,
            torch.from_numpy(sample["right"].transpose(2, 0, 1).copy()).float() / 255,
            torch.from_numpy(sample["disparity"].copy()),
            torch.from_numpy(sample["valid"].copy()),
            torch.from_numpy(sample["segmentation"].copy()),
        )


def atomic_checkpoint(value, path):
    path = Path(path)
    temporary = path.with_suffix(".tmp")
    torch.save(value, temporary)
    os.replace(temporary, path)


def dataset_fingerprint(root):
    fingerprint = hashlib.sha256()
    for filename in ("manifest.json", "data-integrity.json"):
        path = Path(root) / filename
        if path.exists():
            fingerprint.update(path.read_bytes())
    return fingerprint.hexdigest()


def restore_model(checkpoint, device="cpu"):
    # These checkpoints are locally created trusted training artifacts.
    value = torch.load(checkpoint, map_location=device, weights_only=False)
    model = CompactStereo(**value["model_config"]).to(device)
    model.load_state_dict(value["model"])
    model.eval()
    return model, value


def train(config, dataset, output, *, resume=None, epochs=None, overfit=None):
    seed = config["seed"]
    cfg = config["train"]
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.set_num_threads(cfg["threads"])
    device = torch.device(cfg["device"])
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError(
            "Configured CUDA device is unavailable. Select train.device=cpu explicitly."
        )
    torch.use_deterministic_algorithms(True)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    if not resume and (output / "last.pt").exists():
        raise ValueError("Choose a new run directory or explicitly resume its checkpoint")
    training = StereoDataset(dataset, "train", limit=overfit)
    validation = training if overfit else StereoDataset(dataset, "validation")
    manifest = training.manifest
    if manifest.get("config", {}).get("image", config.get("image")) != config.get("image"):
        raise ValueError("Training image configuration must match the captured dataset")
    groups = {
        split: {r["scene_id"] for r in manifest["samples"] if r["split"] == split}
        for split in ("train", "validation", "test", "unseen", "asynchronous")
    }
    for split, ids in groups.items():
        assert all(not ids & other for name, other in groups.items() if name != split)
    loader = DataLoader(training, batch_size=cfg["batch_size"], shuffle=True, num_workers=0)
    validator = DataLoader(validation, batch_size=cfg["batch_size"], shuffle=False, num_workers=0)
    model = CompactStereo(**config["model"]).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=cfg["learning_rate"], weight_decay=1e-4)
    start, best, history = 0, float("inf"), []
    if resume:
        prior = torch.load(resume, map_location=device, weights_only=False)
        if (
            prior["config"] != config
            or prior["dataset_hash"] != dataset_fingerprint(dataset)
            or prior.get("overfit_limit") != overfit
        ):
            raise ValueError("Resume requires the original data manifest and configuration")
        model.load_state_dict(prior["model"])
        optimizer.load_state_dict(prior["optimizer"])
        start, best, history = prior["epoch"] + 1, prior["best"], prior["history"]
        torch.set_rng_state(prior["rng_cpu"].cpu())
        if device.type == "cuda":
            torch.cuda.set_rng_state_all([state.cpu() for state in prior["rng_cuda"]])
        np.random.set_state(prior["rng_numpy"])
        random.setstate(prior["rng_python"])
    if device.type == "cuda":
        torch.cuda.reset_peak_memory_stats()
    begin = time.monotonic()
    previous_elapsed = history[-1]["elapsed_s"] if history else 0
    if (epochs or cfg["epochs"]) <= start:
        raise ValueError("Final epoch must exceed the checkpoint epoch")
    for epoch in range(start, epochs or cfg["epochs"]):
        model.train()
        losses = []
        for left, right, target, mask, segmentation in loader:
            left, right, target, mask, segmentation = (
                x.to(device) for x in (left, right, target, mask, segmentation)
            )
            mask = mask & (target > 0) & (target < config["model"]["max_disparity"])
            prediction, _ = model(left, right)
            error = torch.nn.functional.smooth_l1_loss(prediction[:, 0], target, reduction="none")
            weights = 1 + 4 * ((segmentation > 0) & (segmentation < 4)).float()
            loss = (error[mask] * weights[mask]).sum() / weights[mask].sum().clamp_min(1)
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 10)
            optimizer.step()
            losses.append(float(loss.detach()))
        model.eval()
        errors, foreground = [], []
        with torch.inference_mode():
            for left, right, target, mask, segmentation in validator:
                left, right, target, mask, segmentation = (
                    x.to(device) for x in (left, right, target, mask, segmentation)
                )
                prediction, _ = model(left, right)
                error = (prediction[:, 0] - target).abs()
                mask = mask & (target > 0) & (target < config["model"]["max_disparity"])
                errors.append(float(error[mask].mean()))
                fg = mask & (segmentation > 0) & (segmentation < 4)
                if fg.any():
                    foreground.append(float(error[fg].mean()))
        score = float(np.mean(errors))
        record = dict(
            epoch=epoch,
            loss=float(np.mean(losses)),
            validation_epe_px=score,
            validation_foreground_epe_px=float(np.mean(foreground)),
            elapsed_s=previous_elapsed + time.monotonic() - begin,
        )
        history.append(record)
        improved = score < best
        best = min(score, best)
        checkpoint = dict(
            model=model.state_dict(),
            optimizer=optimizer.state_dict(),
            epoch=epoch,
            best=best,
            config=config,
            model_config=config["model"],
            dataset_hash=dataset_fingerprint(dataset),
            overfit_limit=overfit,
            history=history,
            rng_cpu=torch.get_rng_state(),
            rng_cuda=torch.cuda.get_rng_state_all() if device.type == "cuda" else [],
            rng_numpy=np.random.get_state(),
            rng_python=random.getstate(),
        )
        atomic_checkpoint(checkpoint, output / "last.pt")
        if improved:
            atomic_checkpoint(checkpoint, output / "best.pt")
        (output / "history.json").write_text(json.dumps(history, indent=2))
        print(json.dumps(record), flush=True)
    report = dict(
        device=str(device),
        parameters=sum(p.numel() for p in model.parameters()),
        train_samples=len(training),
        validation_samples=len(validation),
        epochs=len(history),
        best_validation_epe_px=best,
        elapsed_s=previous_elapsed + time.monotonic() - begin,
        session_elapsed_s=time.monotonic() - begin,
        torch_version=torch.__version__,
        gpu=torch.cuda.get_device_name() if device.type == "cuda" else None,
        peak_allocated_mb=torch.cuda.max_memory_allocated() / 2**20 if device.type == "cuda" else 0,
        peak_reserved_mb=torch.cuda.max_memory_reserved() / 2**20 if device.type == "cuda" else 0,
    )
    (output / "training-report.json").write_text(json.dumps(report, indent=2))
    return report
