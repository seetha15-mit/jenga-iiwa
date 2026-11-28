"""The final run's conditions (docs/FINAL_RUN.md), in one place: the submit script, the analysis and the replays all
read them from here, so they cannot disagree.

Each condition: the strategy and the config overrides its games run with (on top of the frozen commit's config.py).
Every override is written to the trial's overrides.txt; the analysis checks those files against this list.
Run as a script, it prints one line per condition for cluster/final_run.sh:  letter strategy NAME=VALUE ...
"""

TRIALS = range(500, 600)  # fresh trials, never used before (docs/FINAL_RUN.md)
COMMON = {"DIAG_LOG_LEVEL": 1}  # the development log (steps.csv): records only (FINAL_RUN.md, Logging)
VARIED = {"FRICTION_MODE": "random"}  # per-block friction 0.3-0.7 (FRICTION_RANGE)
PROBE = {"PROBE_ENABLED": True}  # middle pushes only, at the config's thresholds (D-28)

CONDITIONS = {
    "A": dict(name="random", strategy="random", set={}),
    "B": dict(name="rule-based", strategy="rule_based", set={}),
    "C": dict(name="rule-based + perception", strategy="rule_based", set={"PERCEPTION": True}),
    "D": dict(name="rule-based, varied friction", strategy="rule_based", set=dict(VARIED)),
    "E": dict(name="rule-based + probe", strategy="rule_based", set=dict(VARIED, **PROBE)),
    "F": dict(name="random, varied friction", strategy="random", set=dict(VARIED)),
    "G": dict(name="random + probe", strategy="random", set=dict(VARIED, **PROBE)),
    "H": dict(name="rule-based + probe on all blocks", strategy="rule_based",
              set=dict(VARIED, **PROBE, PROBE_END_PULLS=True, PROBE_EDGE_PUSH=True)),
}

# (id, first, second, kind): kind "difference" (Wilcoxon, Holm across main ones), "equivalence" (TOST +-0.5),
# "secondary" (Wilcoxon, own p value).  The difference is second minus first.
COMPARISONS = [
    ("M1", "A", "B", "difference"),
    ("M2", "B", "C", "equivalence"),
    ("M3", "D", "E", "difference"),
    ("M4", "F", "G", "difference"),
    ("S1", "D", "H", "secondary"),
]
EQUIVALENCE_MARGIN = 0.5  # [user] blocks per game (FINAL_RUN.md: about 8% of a 6-block game)


def overrides(letter: str) -> dict:
    """Every config override for a condition's games (COMMON plus the condition's own)."""
    return dict(COMMON, **CONDITIONS[letter]["set"])


def set_args(letter: str) -> str:
    """The overrides as run_one_trial.py --set arguments (Python literals, no spaces inside a value)."""
    return " ".join(f"{k}={v!r}" for k, v in overrides(letter).items())


if __name__ == "__main__":
    for letter, c in CONDITIONS.items():
        print(letter, c["strategy"], set_args(letter))
