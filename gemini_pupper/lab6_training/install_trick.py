#!/usr/bin/env python3
"""Install a locally trained trick into ~/gemini_pupper/tricks/<name>/ (get_trick.py without W&B).

Same checks get_trick.py runs: the policy must be a trick policy (trick + motion
blocks), a walking policy must not be one, and both must pass the controller's
own validator (pupper_gait_deploy/download_policy.py --validate-only).

    python3 ~/install_trick.py --policy sit_policy.json --yaml sit.trick.yaml --name sit_stay_shake
    python3 ~/install_trick.py --policy clips.json --walk walk.json --yaml spider.trick.yaml --name spider_walk
"""
import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

TRICKS = Path.home() / "gemini_pupper" / "tricks"
VALIDATOR = Path.home() / "pupper_gait_deploy" / "download_policy.py"


def validate(path: Path) -> None:
    if not VALIDATOR.exists():
        sys.exit(f"validator missing: {VALIDATOR}")
    if subprocess.run([sys.executable, str(VALIDATOR), "--validate-only", str(path)]).returncode != 0:
        sys.exit(f"validation failed for {path}; not installing")


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--policy", required=True, type=Path)
    p.add_argument("--walk", type=Path)
    p.add_argument("--yaml", required=True, type=Path)
    p.add_argument("--name", required=True)
    a = p.parse_args()
    policy = json.loads(a.policy.read_text())
    if "trick" not in policy or "motion" not in policy:
        sys.exit(f"{a.policy} is not a trick policy (no trick/motion blocks)")
    validate(a.policy)
    if a.walk:
        if "trick" in json.loads(a.walk.read_text()):
            sys.exit(f"{a.walk} is a trick policy; --walk takes the walking policy")
        validate(a.walk)
    dest = TRICKS / a.name
    dest.mkdir(parents=True, exist_ok=True)
    shutil.copy2(a.policy, dest / "policy.json")
    if a.walk:
        shutil.copy2(a.walk, dest / "walk_policy.json")
    shutil.copy2(a.yaml, dest / "trick.yaml")
    t = policy["trick"]
    print(f"installed {dest}: {t.get('frames')} frames, {t.get('duration_s', 0):.1f} s")
    for c in t.get("clips", []):
        print(f"  clip {c['name']:14} loop={c.get('loop')} next={c.get('next')}")
    print("restart: robot/stop.sh && robot/start.sh")


if __name__ == "__main__":
    main()
