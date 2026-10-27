"""Run trials.   python run_experiment.py [--strategy random] [--trials 10] [--mover fake|real] [--turns 10]

Everything else comes from config.py.  Output goes to runs/ (real mover) or runs_fake/ (fake mover; INVALID results).
Each trial writes a replay you can open in a browser (trial_XX_replay.html) and CSV logs.
"""

import argparse
import sys
from types import SimpleNamespace

import config as _config
from jenga import config_check, mover as mover_mod
from jenga.evaluation import harness


def load_cfg(args):
    cfg = SimpleNamespace(**{k: v for k, v in vars(_config).items() if k.isupper()})
    if args.strategy:
        cfg.STRATEGY = args.strategy
    if args.trials:
        cfg.N_TRIALS = args.trials
    if args.mover:
        cfg.MOVER = args.mover
    if args.turns:
        cfg.MAX_TURNS = args.turns
    return cfg


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--strategy")
    ap.add_argument("--trials", type=int)
    ap.add_argument("--mover", choices=["fake", "real"])
    ap.add_argument("--turns", type=int)
    cfg = load_cfg(ap.parse_args())

    print(config_check.describe(cfg))
    for w in config_check.check_values(cfg):
        print(f"[config] {w}")
    mover = mover_mod.get_mover(cfg.MOVER)
    out_dir = harness.run_directory(cfg, mover, cfg.STRATEGY)
    print(f"\nWriting to {out_dir}\n")
    if cfg.RECORD_REPLAY:
        print("Note: nothing is shown live. Watch the trial_XX_replay.html files in that folder afterwards.\n")
    try:
        for trial in range(cfg.N_TRIALS):
            harness.run_trial(cfg, trial, mover, out_dir)
    except ImportError as e:
        print(f"\nSTOPPED: {e}\nDrake is not available in this Python. Activate the course environment (docs/SETUP.md, step 4).")
        sys.exit(1)
    except NotImplementedError as e:
        print(f"\nSTOPPED: {e}\nThe team's steps are not written yet. Use  --mover fake  to test the harness "
              "(its results are INVALID), or write them (docs/FOR_THE_TEAM.md).")
        sys.exit(1)
    print(f"\nDone.  Files are in {out_dir}.  Open the *_replay.html files in a browser to watch.")
    if mover.TAG == "fake":
        print("These were FAKE runs: they are in the fake-runs folder and cannot be used for results.")


if __name__ == "__main__":
    main()
