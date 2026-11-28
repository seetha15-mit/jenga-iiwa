"""The final run's analysis, as fixed in docs/FINAL_RUN.md: validity checks, the comparisons (Wilcoxon with Holm,
TOST equivalence, a secondary test) and the descriptive tables.

Usage:  python cluster/analyze_final.py <root> [--commit HASH] [--no-checks] [--trials 500-599]
<root>/<letter>/<strategy>_trial<NNN>/ holds each game's trials.csv, turns.csv, steps.csv, probes.csv and
overrides.txt; <root>/<letter>/code.txt names the commit.  Conditions and comparisons come from
cluster/final_conditions.py, never from here.  Writes <root>/analysis/report.md, comparisons.csv, per_condition.csv.
Reads simulator truth only through the logs: evaluation side (CLAUDE.md rule 5).
"""

import argparse
import ast
import csv
import sys
from pathlib import Path

import numpy as np
from scipy import stats

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # the repo root: config and cluster.final_conditions
from cluster import final_conditions as fc  # noqa: E402

FINGER_MARGIN_MM = 0.875  # [derived] the finger height margin per side (D-27): perception error over it matters


# ---------------------------------------------------------------- reading
def parse_overrides(text: str) -> dict:
    """overrides.txt ("NAME = repr(value)" per line, or "none") -> {name: value}."""
    out = {}
    for line in text.splitlines():
        line = line.strip()
        if not line or line == "none":
            continue
        name, _, value = line.partition(" = ")
        out[name.strip()] = ast.literal_eval(value.strip())
    return out


def check_overrides(found: dict, expected: dict) -> list:
    """Differences between a game's overrides and its condition's, as readable lines ([] = they match)."""
    msgs = []
    for k in sorted(set(found) | set(expected)):
        if k not in found:
            msgs.append(f"{k} missing (expected {expected[k]!r})")
        elif k not in expected:
            msgs.append(f"{k} = {found[k]!r} not in the condition")
        elif found[k] != expected[k]:
            msgs.append(f"{k} = {found[k]!r}, expected {expected[k]!r}")
    return msgs


def _csv(path: Path) -> list:
    if not path.exists():
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def load_condition(cdir: Path) -> dict:
    """{trial: game} for every finished game (a trials.csv row) under one condition's folder."""
    games = {}
    for tdir in sorted(p for p in cdir.iterdir() if p.is_dir() and (p / "trials.csv").exists()):
        rows = _csv(tdir / "trials.csv")
        if not rows:
            continue
        t = rows[0]
        games[int(t["trial"])] = dict(
            folder=tdir, blocks=int(t["blocks_replaced"]), wall=float(t["wall_seconds"] or 0), trial_row=t,
            turns=_csv(tdir / "turns.csv"), probes=_csv(tdir / "probes.csv"),
            overrides=parse_overrides((tdir / "overrides.txt").read_text()) if (tdir / "overrides.txt").exists() else None)
    return games


def read_commit(cdir: Path):
    """The commit hash in a condition's code.txt (first word), or None."""
    p = cdir / "code.txt"
    return p.read_text().split()[0] if p.exists() and p.read_text().split() else None


# ---------------------------------------------------------------- validity
def validity(root: Path, data: dict, trials, commit=None) -> tuple:
    """(problems that stop the analysis, notes): overrides against final_conditions, one commit for all, missing
    trials per condition (a missing game is reported, not fatal: FINAL_RUN.md reruns it once, then leaves it out)."""
    problems, notes, commits = [], [], {}
    for letter, games in data.items():
        expected, by_msg = fc.overrides(letter), {}
        for t, g in sorted(games.items()):
            msgs = ["no overrides.txt"] if g["overrides"] is None else check_overrides(g["overrides"], expected)
            for m in msgs:
                by_msg.setdefault(m, []).append(t)
        for m, ts in by_msg.items():  # one line per problem, with the games it affects
            problems.append(f"{letter} trial {ts[0]}: {m}" if len(ts) == 1 else f"{letter} trials {_ranges(ts)}: {m}")
        commits[letter] = read_commit(root / letter)
        missing = sorted(set(trials) - set(games))
        if missing:
            notes.append(f"{letter}: {len(missing)} game(s) missing: {_ranges(missing)}")
    found = {c for c in commits.values()}
    if None in found:
        problems.append("no code.txt for " + ", ".join(k for k, v in commits.items() if v is None))
    named = found - {None}
    if len(named) > 1:
        problems.append("conditions ran on different commits: " + ", ".join(f"{k} {v}" for k, v in commits.items()))
    if commit and named and named != {commit} and not all(c.startswith(commit) for c in named):
        problems.append(f"code.txt commit(s) {sorted(named)} are not the frozen commit {commit}")
    return problems, notes


def _ranges(xs) -> str:
    out, start, prev = [], None, None
    for x in xs:
        if start is None:
            start = prev = x
        elif x == prev + 1:
            prev = x
        else:
            out.append(f"{start}-{prev}" if prev > start else f"{start}")
            start = prev = x
    if start is not None:
        out.append(f"{start}-{prev}" if prev > start else f"{start}")
    return ", ".join(out)


# ---------------------------------------------------------------- statistics
def paired(a: dict, b: dict) -> tuple:
    """(trials in both, differences b - a in blocks per game, trials left out)."""
    both = sorted(set(a) & set(b))
    d = np.array([b[t]["blocks"] - a[t]["blocks"] for t in both], dtype=float)
    return both, d, len(set(a) ^ set(b))


def mean_ci(d, conf: float = 0.95) -> tuple:
    """Mean of d and its t-based confidence interval (lo, hi)."""
    d = np.asarray(d, float)
    m = float(d.mean())
    if len(d) < 2 or d.std(ddof=1) == 0:
        return m, m, m
    h = stats.t.ppf(0.5 + conf / 2, len(d) - 1) * d.std(ddof=1) / np.sqrt(len(d))
    return m, m - h, m + h


def wilcoxon_p(d) -> float:
    """Two-sided Wilcoxon signed-rank p on paired differences (zeros dropped, scipy's default); 1.0 if all are zero."""
    d = np.asarray(d, float)
    if not d.any():
        return 1.0
    return float(stats.wilcoxon(d).pvalue)


def holm(pvals) -> list:
    """Holm-adjusted p values, in the input order (each at most 1, never smaller than a more significant one's)."""
    p = np.asarray(pvals, float)
    order = np.argsort(p)
    adj, running = np.empty_like(p), 0.0
    for rank, i in enumerate(order):
        running = max(running, (len(p) - rank) * p[i])
        adj[i] = min(running, 1.0)
    return adj.tolist()


def tost(d, margin: float) -> dict:
    """Paired equivalence test (two one-sided t-tests) of mean d within +-margin.  Equivalent if both one-sided p
    are below 0.05, the same as the 90% CI lying inside +-margin."""
    d = np.asarray(d, float)
    n, m = len(d), float(d.mean())
    se = d.std(ddof=1) / np.sqrt(n) if n > 1 else 0.0
    if se == 0:
        p_low = 0.0 if m > -margin else 1.0
        p_high = 0.0 if m < margin else 1.0
    else:
        p_low = float(stats.t.sf((m + margin) / se, n - 1))  # H0: mean <= -margin
        p_high = float(stats.t.cdf((m - margin) / se, n - 1))  # H0: mean >= +margin
    _, lo, hi = mean_ci(d, 0.90)
    return dict(p_low=p_low, p_high=p_high, p=max(p_low, p_high), ci90=(lo, hi), equivalent=max(p_low, p_high) < 0.05)


def counts(d) -> tuple:
    """(better, worse, same): games where the second condition placed more, fewer, the same blocks."""
    d = np.asarray(d)
    return int((d > 0).sum()), int((d < 0).sum()), int((d == 0).sum())


def compare(data: dict) -> list:
    """One row per COMPARISON whose two conditions are present (others skipped with a note in the row list)."""
    rows = []
    for cid, a, b, kind in fc.COMPARISONS:
        if a not in data or b not in data:
            rows.append(dict(id=cid, first=a, second=b, kind=kind, skipped=f"{a if a not in data else b} not run"))
            continue
        both, d, left_out = paired(data[a], data[b])
        m, lo, hi = mean_ci(d)
        better, worse, same = counts(d)
        r = dict(id=cid, first=a, second=b, kind=kind, n=len(both), left_out=left_out,
                 mean_first=float(np.mean([data[a][t]["blocks"] for t in both])),
                 mean_second=float(np.mean([data[b][t]["blocks"] for t in both])),
                 diff=m, ci95_lo=lo, ci95_hi=hi, better=better, worse=worse, same=same,
                 early_only_first=sum(1 for t in both if data[a][t]["blocks"] <= 1 < data[b][t]["blocks"]),
                 early_only_second=sum(1 for t in both if data[b][t]["blocks"] <= 1 < data[a][t]["blocks"]))
        if kind == "equivalence":
            e = tost(d, fc.EQUIVALENCE_MARGIN)
            r.update(p=e["p"], p_low=e["p_low"], p_high=e["p_high"], ci90_lo=e["ci90"][0], ci90_hi=e["ci90"][1],
                     claim="equivalent" if e["equivalent"] else "not shown equivalent")
        else:
            r["p"] = wilcoxon_p(d)
        rows.append(r)
    main = [r for r in rows if r.get("kind") == "difference" and "p" in r]
    for r, p in zip(main, holm([r["p"] for r in main])):
        r["p_holm"] = p
        r["claim"] = "difference" if p < 0.05 else "no difference found"
    for r in rows:
        if r.get("kind") == "secondary" and "p" in r:
            r["claim"] = "difference (secondary, own p)" if r["p"] < 0.05 else "no difference found (secondary)"
    return rows


# ---------------------------------------------------------------- descriptive
def end_step(steps: list, turn: str) -> str:
    """Where a turn went wrong: the first step (not a waypoint row) with an error, else the step that moved the other
    blocks most (worst_mm).  '' when the turn has no step rows (development log off)."""
    rows = [s for s in steps if s.get("turn") == turn and not s.get("waypoint")]
    for s in rows:
        if s.get("error"):
            return s["step"]
    if not rows:
        return ""
    return max(rows, key=lambda s: float(s.get("worst_mm") or 0))["step"]


def describe(games: dict, max_per_turn: int) -> dict:
    """Per-condition summary (FINAL_RUN.md, 'Also reported')."""
    b = np.array([g["blocks"] for g in games.values()])
    trials = sorted(games)
    out = dict(n=len(b), mean=float(b.mean()), median=float(np.median(b)), sd=float(b.std(ddof=1)) if len(b) > 1 else 0.0,
               six_plus=int((b >= 6).sum()), early=int((b <= 1).sum()),
               longest=max(trials, key=lambda t: (games[t]["blocks"], -t)),
               hist={int(k): int(v) for k, v in zip(*np.unique(b, return_counts=True))},
               wall_min=float(np.mean([g["wall"] for g in games.values()]) / 60))
    out["longest_blocks"] = games[out["longest"]]["blocks"]
    ends, middles, nturns = {}, 0, 0
    for t in trials:
        g = games[t]
        turns = g["turns"]
        nturns += len(turns)
        middles += sum(1 for r in turns if r.get("slot") == "1")
        if not turns:
            continue
        last = turns[-1]
        steps = _csv(g["folder"] / "steps.csv")  # read only here: the one big file
        collapsed = last.get("collapse") == "True" and last["outcome"] != "collapse"
        key = (f"{last['outcome']}{' +collapse' if collapsed else ''}",
               "push" if last.get("slot") == "1" else "end-pinch", last.get("layer"), end_step(steps, last["turn"]))
        ends[key] = ends.get(key, 0) + 1
    out["ends"] = sorted(ends.items(), key=lambda kv: -kv[1])
    out["middle_share"] = middles / nturns if nturns else 0.0
    # perception (C): turns.csv columns; empty when PERCEPTION was off
    pos = [float(r["perception_max_pos_mm"]) for g in games.values() for r in g["turns"] if r.get("perception_max_pos_mm")]
    if pos:
        hgt = [float(r["perception_max_height_mm"]) for g in games.values() for r in g["turns"] if r.get("perception_max_height_mm")]
        unseen = sum(int(r["perception_unseen"] or 0) for g in games.values() for r in g["turns"] if r.get("perception_unseen"))
        out["perception"] = dict(turns=len(pos), max_pos_mm=max(pos), median_pos_mm=float(np.median(pos)),
                                 max_height_mm=max(hgt) if hgt else 0.0, unseen=unseen,
                                 over_margin=sum(1 for x in pos if x > FINGER_MARGIN_MM))
    probes = [(t, p) for t, g in games.items() for p in g["probes"]]
    if probes:
        acc = [p for _, p in probes if p["accepted"] == "True"]
        rej = [p for _, p in probes if p["accepted"] != "True"]
        # a turn that used up its rejections took the least-resisting block unprobed (harness); counting turns with
        # probes_rejected == PROBE_MAX_PER_TURN approximates the fallbacks (it misses a fallback after running out
        # of candidates with fewer rejections)
        fallbacks = sum(1 for g in games.values() for r in g["turns"] if int(r.get("probes_rejected") or 0) >= max_per_turn)
        rej_turns = {(t, p["turn"]) for t, p in probes if p["accepted"] != "True"}
        knocked = sum(1 for t, g in games.items() if g["turns"] and g["turns"][-1].get("collapse") == "True"
                      and (t, g["turns"][-1]["turn"]) in rej_turns)

        def res(rows, kind):
            x = [float(p["resistance_n"]) for p in rows if (p["slot"] == "1") == (kind == "middle")]
            return (len(x), float(np.median(x)) if x else float("nan"))

        out["probe"] = dict(per_game=len(probes) / len(games), rejected=len(rej), fallbacks=fallbacks,
                            collapse_on_rejection_turn=knocked,
                            accepted_edge=res(acc, "edge"), accepted_middle=res(acc, "middle"),
                            rejected_edge=res(rej, "edge"), rejected_middle=res(rej, "middle"))
    return out


# ---------------------------------------------------------------- output
def _f(x, nd=2):
    return f"{x:.{nd}f}" if isinstance(x, float) else str(x)


def report(data: dict, rows: list, desc: dict, notes: list) -> str:
    """The readable report (markdown)."""
    L = ["# Final run: analysis", ""]
    if notes:
        L += ["## Notes", ""] + [f"- {n}" for n in notes] + [""]
    L += ["## Comparisons (blocks per game, second minus first)", ""]
    for r in rows:
        if "skipped" in r:
            L.append(f"- {r['id']} {r['first']} vs {r['second']}: skipped ({r['skipped']})")
            continue
        head = (f"- **{r['id']}** {fc.CONDITIONS[r['second']]['name']} ({r['second']}) vs "
                f"{fc.CONDITIONS[r['first']]['name']} ({r['first']}), n {r['n']}"
                + (f" ({r['left_out']} left out)" if r["left_out"] else "") + ": "
                f"{r['mean_first']:.2f} -> {r['mean_second']:.2f}, diff {r['diff']:+.2f} "
                f"(95% CI {r['ci95_lo']:+.2f} to {r['ci95_hi']:+.2f}); better/worse/same {r['better']}/{r['worse']}/{r['same']}")
        if r["kind"] == "equivalence":
            head += (f"; TOST +-{fc.EQUIVALENCE_MARGIN}: p {r['p_low']:.4f} / {r['p_high']:.4f}, 90% CI "
                     f"{r['ci90_lo']:+.2f} to {r['ci90_hi']:+.2f}: **{r['claim']}**")
        elif r["kind"] == "difference":
            head += f"; Wilcoxon p {r['p']:.4f}, Holm {r['p_holm']:.4f}: **{r['claim']}**"
        else:
            head += f"; Wilcoxon p {r['p']:.4f}: **{r['claim']}**"
        head += f"; early failures only in {r['first']}: {r['early_only_first']}, only in {r['second']}: {r['early_only_second']}"
        L.append(head)
    L += ["", "## Per condition", ""]
    for letter, d in desc.items():
        L.append(f"### {letter}: {fc.CONDITIONS[letter]['name']}")
        L.append(f"- n {d['n']}, mean {d['mean']:.2f}, median {d['median']:.1f}, SD {d['sd']:.2f}; 6+ blocks {d['six_plus']}; "
                 f"longest {d['longest_blocks']} (trial {d['longest']}); early failures (<= 1 block) {d['early']}; "
                 f"middles {d['middle_share'] * 100:.0f}% of turns; wall {d['wall_min']:.1f} min per game")
        L.append("- blocks per game: " + ", ".join(f"{k}: {v}" for k, v in d["hist"].items()))
        L.append("- how games end (outcome, method, layer, step): " +
                 "; ".join(f"{o} {m} L{lay} {s or '-'} x{c}" for (o, m, lay, s), c in d["ends"][:8]))
        if "perception" in d:
            p = d["perception"]
            L.append(f"- perception: {p['turns']} turns, worst block error median {p['median_pos_mm']:.2f} mm, max "
                     f"{p['max_pos_mm']:.1f} mm (height max {p['max_height_mm']:.1f} mm); over {FINGER_MARGIN_MM} mm in "
                     f"{p['over_margin']} turns; unseen block-turns {p['unseen']}")
        if "probe" in d:
            p = d["probe"]
            L.append(f"- probe: {p['per_game']:.1f} probes per game, {p['rejected']} rejected, ~{p['fallbacks']} fallbacks, "
                     f"{p['collapse_on_rejection_turn']} games ended on a turn with a rejection; median N accepted "
                     f"edge {_f(p['accepted_edge'][1])} (n {p['accepted_edge'][0]}) middle {_f(p['accepted_middle'][1])} "
                     f"(n {p['accepted_middle'][0]}), rejected edge {_f(p['rejected_edge'][1])} (n {p['rejected_edge'][0]}) "
                     f"middle {_f(p['rejected_middle'][1])} (n {p['rejected_middle'][0]})")
        L.append("")
    return "\n".join(L)


def write_csv(path: Path, rows: list) -> None:
    cols = []
    for r in rows:
        cols += [k for k in r if k not in cols]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)


def parse_trials(text: str):
    a, _, b = text.partition("-")
    return range(int(a), int(b or a) + 1)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("root")
    ap.add_argument("--commit")
    ap.add_argument("--no-checks", action="store_true")
    ap.add_argument("--trials", help="e.g. 0-99 (default: final_conditions.TRIALS)")
    a = ap.parse_args(argv)
    root = Path(a.root)
    trials = parse_trials(a.trials) if a.trials else fc.TRIALS
    data, notes = {}, []
    for letter in fc.CONDITIONS:
        if (root / letter).is_dir():
            data[letter] = {t: g for t, g in load_condition(root / letter).items() if t in trials}
        else:
            notes.append(f"{letter} ({fc.CONDITIONS[letter]['name']}): no folder, skipped")
    problems, more = validity(root, data, trials, a.commit)
    notes += more
    if problems and not a.no_checks:
        print(f"STOPPED: {len(problems)} validity problem(s) (FINAL_RUN.md, Validity rules):")
        for p in problems[:40]:
            print("  " + p)
        if len(problems) > 40:
            print(f"  ... and {len(problems) - 40} more")
        return 1
    if problems:
        notes.append(f"--no-checks: {len(problems)} validity problem(s) ignored (first: {problems[0]})")
    import config  # PROBE_MAX_PER_TURN for the fallback count
    rows = compare(data)
    desc = {letter: describe(g, config.PROBE_MAX_PER_TURN) for letter, g in data.items() if g}
    text = report(data, rows, desc, notes)
    print(text)
    out = root / "analysis"
    out.mkdir(exist_ok=True)
    (out / "report.md").write_text(text)
    write_csv(out / "comparisons.csv", rows)
    write_csv(out / "per_condition.csv", [dict(condition=k, **{c: v for c, v in d.items() if not isinstance(v, (dict, list))})
                                          for k, d in desc.items()])
    print(f"\nWrote {out / 'report.md'}, comparisons.csv, per_condition.csv")
    return 0


if __name__ == "__main__":
    sys.exit(main())
