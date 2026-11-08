"""Run ONE trial.  python run_one_trial.py --strategy random --trial 3 --out-root /path/to/runs"""
import argparse
import sys
from pathlib import Path
from types import SimpleNamespace

import config as _config
from jenga import mover as mover_mod
from jenga.evaluation import harness

ap = argparse.ArgumentParser()
ap.add_argument("--strategy", required=True)
ap.add_argument("--trial", type=int, required=True)
ap.add_argument("--mover", default="real", choices=["fake", "real"])
ap.add_argument("--turns", type=int)
ap.add_argument("--out-root", required=True)
a = ap.parse_args()

cfg = SimpleNamespace(**{k: v for k, v in vars(_config).items() if k.isupper()})
cfg.STRATEGY, cfg.MOVER, cfg.RECORD_REPLAY = a.strategy, a.mover, False
if a.turns:
    cfg.MAX_TURNS = a.turns

out_dir = Path(a.out_root) / f"{a.strategy}_trial{a.trial:02d}"  # same name every time, so reruns don't duplicate
if (out_dir / "trials.csv").exists():
    print(f"{out_dir} already done, skipping")
    sys.exit(0)
out_dir.mkdir(parents=True, exist_ok=True)
for stale in ("turns.csv", "trials.csv"):  # clear a partial run
    (out_dir / stale).unlink(missing_ok=True)
cfg.ASSET_DIR = str(out_dir / "assets")  # per-trial SDF files

try:
    harness.run_trial(cfg, a.trial, mover_mod.get_mover(cfg.MOVER), out_dir)
except (ImportError, NotImplementedError) as e:
    print(f"STOPPED: {e}")
    sys.exit(1)