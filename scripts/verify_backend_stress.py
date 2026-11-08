"""Repeat real DDS startup/isolation; unexpected restarts never count as a clean round."""

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rounds", type=int, default=12)
    args = parser.parse_args()
    assert args.rounds >= 2
    cache = Path(".bootstrap/bridge-stress") / str(time.time_ns())
    cache.mkdir(parents=True)
    rounds = []
    environment = dict(os.environ, QT_QPA_PLATFORM="offscreen")
    environment.pop("PYTHONPATH", None)
    for index in range(args.rounds):
        mode = "overlapping" if index % 2 == 0 else "sequential"
        path = cache / f"round-{index + 1:02}.json"
        result = subprocess.run(
            [
                sys.executable,
                "scripts/verify_ros_domains.py",
                "--start-mode",
                mode,
                "--report",
                str(path),
            ],
            env=environment,
            text=True,
            capture_output=True,
            timeout=100,
        )
        (cache / f"round-{index + 1:02}.log").write_text(result.stdout + result.stderr)
        data = json.loads(path.read_text()) if path.exists() else {}
        health = data.get("runtime_health", {})
        restarts = sum(row["restarts"] for row in health.values())
        exits = sum(len(row["unexpected_exits"]) for row in health.values())
        clean = result.returncode == 0 and restarts == 0 and exits == 0
        rounds.append(
            dict(
                round=index + 1,
                mode=mode,
                passed=clean,
                returncode=result.returncode,
                unexpected_restarts=restarts,
                unexpected_exits=exits,
                domain_result=data,
            )
        )
        print(f"ROUND {index + 1}/{args.rounds} {mode}: {'PASS' if clean else 'FAIL'}", flush=True)
    report = dict(
        rounds=rounds,
        passed=sum(r["passed"] for r in rounds),
        failed=sum(not r["passed"] for r in rounds),
        unexpected_restarts=sum(r["unexpected_restarts"] for r in rounds),
        hardware_access=False,
        crash_recovery_is_not_hidden=True,
    )
    Path("docs/backend-stress-verification.json").write_text(json.dumps(report, indent=2) + "\n")
    assert report["failed"] == 0, f"Stress failures; original logs retained in {cache}"


if __name__ == "__main__":
    main()
