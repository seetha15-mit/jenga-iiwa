"""Build the results table from finished runs.  Run:  python make_results.py [runs_folder]

REFUSES to build a table if anything it reads came from the fake mover: either the folder is the
fake-runs folder, or any trials.csv row has mover != "real".  This is the second of the two guards that
keep teleported runs out of the report (the first is that fake runs are written to a separate folder).
"""

import csv
import statistics
from pathlib import Path


class FakeRunsError(RuntimeError):
    pass


def read_trials(runs_dir, cfg=None):
    runs_dir = Path(runs_dir)
    if cfg is not None and runs_dir.name == Path(cfg.FAKE_RUNS_DIR).name:
        raise FakeRunsError(f"{runs_dir} is the fake-runs folder.  Fake runs are never used for results.")
    rows = []
    for path in sorted(runs_dir.glob("*/trials.csv")):
        with open(path, newline="") as f:
            for r in csv.DictReader(f):
                r["_file"] = str(path)
                rows.append(r)
    fake = [r for r in rows if r.get("mover") != "real"]
    if fake:
        raise FakeRunsError(
            f"{len(fake)} trial row(s) are not from the real mover (first: {fake[0]['_file']}). "
            "Results from the fake mover are INVALID.  Remove those runs or move them to the fake-runs folder."
        )
    return rows


def build_table(rows):
    """Group trials by (strategy, friction_mode) and summarise blocks_replaced."""
    groups = {}
    for r in rows:
        groups.setdefault((r["strategy"], r["friction_mode"]), []).append(r)
    table = []
    for (strategy, friction), rs in sorted(groups.items()):
        n = [int(r["blocks_replaced"]) for r in rs]
        collapsed = sum(1 for r in rs if r["collapsed"] == "True")
        table.append(dict(
            strategy=strategy, friction_mode=friction, trials=len(rs), mean_blocks=round(statistics.mean(n), 2),
            median_blocks=statistics.median(n), min_blocks=min(n), max_blocks=max(n),
            sd_blocks=round(statistics.pstdev(n), 2), collapse_rate=round(collapsed / len(rs), 2),
            reached_4=sum(1 for v in n if v >= 4),
        ))
    return table


def format_table(table) -> str:
    cols = ["strategy", "friction_mode", "trials", "mean_blocks", "median_blocks", "min_blocks", "max_blocks",
            "sd_blocks", "collapse_rate", "reached_4"]
    widths = [max(len(c), *(len(str(t[c])) for t in table)) if table else len(c) for c in cols]
    lines = ["  ".join(c.ljust(w) for c, w in zip(cols, widths))]
    for t in table:
        lines.append("  ".join(str(t[c]).ljust(w) for c, w in zip(cols, widths)))
    lines.append("")
    lines.append("blocks = blocks extracted AND replaced before collapse.  reached_4 = trials that met the course minimum of 4.")
    return "\n".join(lines)


def main(runs_dir, cfg):
    rows = read_trials(runs_dir, cfg)
    if not rows:
        print(f"No finished trials found under {runs_dir}.")
        return 1
    table = build_table(rows)
    print(format_table(table))
    out = Path("results")
    out.mkdir(exist_ok=True)
    with open(out / "summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(table[0].keys()))
        w.writeheader()
        w.writerows(table)
    print(f"\nWrote {out / 'summary.csv'}")
    return 0
