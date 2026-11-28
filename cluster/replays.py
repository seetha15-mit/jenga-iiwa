"""Replays for the final run (docs/FINAL_RUN.md, Replays): choose the games by the plan's rules, rerun them with
replays on, check each rerun matches its original, and write the manifest replays.csv.

    python cluster/replays.py select <root>     choose games -> <root>/replays/selection.csv
    python cluster/replays.py submit <root>     rerun them on the cluster (from the frozen worktree)
    python cluster/replays.py verify <root>     compare with the originals -> <root>/replays/replays.csv

<root> is the final run's folder: <root>/<letter>/<strategy>_trial<NNN>/ (cluster/final_run.sh).  Games repeat
exactly only on the same platform and container (O-5), so replays are made on the cluster and shared as files with
the manifest, which records everything needed to make them again.
"""

import csv
import hashlib
import os
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import final_conditions as fc  # noqa: E402


def read_blocks(cond_dir: Path) -> dict:
    """{trial: trials.csv row} for one condition's folder."""
    out = {}
    for f in sorted(Path(cond_dir).glob("*/trials.csv")):
        for r in csv.DictReader(open(f)):
            out[int(r["trial"])] = r
    return out


def pick_condition(blocks: dict) -> list:
    """Best (most blocks), worst (fewest) and typical (closest to the median) of {trial: blocks}; ties go to the
    lowest trial.  Returns [(reason, trial)]."""
    trials = sorted(blocks)
    vals = sorted(blocks[t] for t in trials)
    n = len(vals)
    median = (vals[(n - 1) // 2] + vals[n // 2]) / 2
    best = min(trials, key=lambda t: (-blocks[t], t))
    worst = min(trials, key=lambda t: (blocks[t], t))
    typical = min(trials, key=lambda t: (abs(blocks[t] - median), t))
    return [("best", best), ("worst", worst), ("typical", typical)]


def pick_pairs(first: dict, second: dict) -> list:
    """The paired trial where `second` gained most over `first`, and the one where it lost most (ties: lowest
    trial).  Returns [(reason, trial)]."""
    common = sorted(set(first) & set(second))
    diff = {t: second[t] - first[t] for t in common}
    gain = min(common, key=lambda t: (-diff[t], t))
    loss = min(common, key=lambda t: (diff[t], t))
    return [(f"gained most ({diff[gain]:+d})", gain), (f"lost most ({diff[loss]:+d})", loss)]


def selection(root: Path) -> list:
    """Every game to replay: [(letter, trial, reasons)], one rerun per game even when chosen for several reasons."""
    rows = {L: read_blocks(root / L) for L in fc.CONDITIONS if (root / L).is_dir()}
    blocks = {L: {t: int(r["blocks_replaced"]) for t, r in rs.items()} for L, rs in rows.items()}
    chosen = {}
    for L, b in blocks.items():
        if b:
            for reason, t in pick_condition(b):
                chosen.setdefault((L, t), []).append(f"{L} {reason}")
    for cid, a, b, _ in fc.COMPARISONS:
        if blocks.get(a) and blocks.get(b):
            for reason, t in pick_pairs(blocks[a], blocks[b]):
                for L in (a, b):
                    chosen.setdefault((L, t), []).append(f"{cid} ({b} vs {a}) {reason}")
    return [(L, t, "; ".join(rs)) for (L, t), rs in sorted(chosen.items())]


def command(letter: str, trial: int, out_root: str) -> str:
    """The exact command that makes one replay (inside the container, from the frozen worktree)."""
    c = fc.CONDITIONS[letter]
    return (f"apptainer exec jenga.sif python run_one_trial.py --strategy {c['strategy']} --trial {trial} --mover real "
            f"--out-root {out_root} --replay --set {fc.set_args(letter)}")


def submit(root: Path) -> None:
    """One Slurm array per condition over its chosen trials, replays on, into <root>/replays/<letter>."""
    sel = list(csv.DictReader(open(root / "replays" / "selection.csv")))
    for L in sorted({r["condition"] for r in sel}):
        trials = ",".join(r["trial"] for r in sel if r["condition"] == L)
        out = root / "replays" / L
        out.mkdir(parents=True, exist_ok=True)
        env = dict(os.environ, OUT_ROOT=str(out), STRATEGY=fc.CONDITIONS[L]["strategy"], REPLAY="1",
                   SET=fc.set_args(L))
        job = subprocess.run(["sbatch", "--parsable", "-J", f"replay_{L}", "-t", "03:00:00", "-a", trials,
                              "cluster/jenga_array.sbatch"], env=env, check=True, capture_output=True, text=True)
        print(L, trials, "job", job.stdout.strip())


def sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()[:16] if Path(path).exists() else ""


def verify(root: Path) -> int:
    """Compare every rerun with its original (blocks, end, collapse) and write the manifest.  Returns the number of
    reruns that do not match or are missing (they do not count, FINAL_RUN.md)."""
    sel = list(csv.DictReader(open(root / "replays" / "selection.csv")))
    lock = sha256(Path(__file__).resolve().parent / "requirements.lock")
    out, bad = [], 0
    for r in sel:
        L, t = r["condition"], int(r["trial"])
        orig = read_blocks(root / L).get(t)
        rerun = read_blocks(root / "replays" / L).get(t)
        code = (root / L / "code.txt").read_text().split()[0] if (root / L / "code.txt").exists() else ""
        same = bool(orig and rerun and all(orig[k] == rerun[k] for k in ("blocks_replaced", "end_reason", "collapsed")))
        bad += not same
        out.append(dict(reasons=r["reasons"], condition=L, condition_name=fc.CONDITIONS[L]["name"],
                        strategy=fc.CONDITIONS[L]["strategy"], trial=t, seed=orig["seed"] if orig else "",
                        overrides=fc.set_args(L), commit=code, requirements_lock_sha256=lock,
                        command=command(L, t, str(root / "replays" / L)),
                        blocks_original=orig["blocks_replaced"] if orig else "",
                        blocks_rerun=rerun["blocks_replaced"] if rerun else "", matches=same,
                        replay_file=rerun["replay_file"] if rerun else ""))
    path = root / "replays" / "replays.csv"
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(out[0]))
        w.writeheader()
        w.writerows(out)
    print(f"{len(out)} replays, {bad} not matching or missing; manifest {path}")
    return bad


def main(argv) -> int:
    mode, root = argv[1], Path(argv[2]).resolve()
    if mode == "select":
        sel = selection(root)
        (root / "replays").mkdir(parents=True, exist_ok=True)
        with open(root / "replays" / "selection.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["condition", "trial", "reasons"])
            w.writerows(sel)
        for row in sel:
            print(*row)
        print(f"{len(sel)} games -> {root / 'replays' / 'selection.csv'}")
        return 0
    if mode == "submit":
        submit(root)
        return 0
    if mode == "verify":
        return 1 if verify(root) else 0
    print(__doc__)
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
