"""Score a finished trick run: newest checkpoint + policy.json for RUN, then play_motion's verdict.
usage (on the Colab VM): python score.py RUN_NAME MOTION.npz   (MOTION "-" for a walking policy: no score)"""
import glob
import os
import re
import subprocess
import sys

run, motion = sys.argv[1], sys.argv[2]
dirs = [d for d in glob.glob("/content/pupper-mjlab/logs/rsl_rl/*/*") if d.endswith(run)]
if not dirs:
    sys.exit(f"no run dir ending in {run}")
d = max(dirs, key=os.path.getmtime)
cks = sorted(glob.glob(f"{d}/model_*.pt"), key=lambda p: int(re.findall(r"model_(\d+)", p)[0]))
pj = f"{d}/policy.json"
print("RUN_DIR", d)
print("CHECKPOINT", cks[-1] if cks else None)
print("POLICY_JSON", pj if os.path.exists(pj) else "MISSING")
if motion != "-" and cks:
    r = subprocess.run(
        f"cd /content/pupper-mjlab && timeout 900 python -m mjlab.tasks.tracking.config.pupper.play_motion {motion} --checkpoint {cks[-1]}",
        shell=True, capture_output=True, text=True)
    lines = [l for l in r.stdout.splitlines() if re.search(r"worst|verdict|HOLDS|WOBBLES|FALLS|FELL|^\s+\S+\s*$", l)]
    print("\n".join(lines[-12:]) or r.stdout[-2000:] + r.stderr[-1500:])
