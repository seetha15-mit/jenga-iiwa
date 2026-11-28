# Final run: results

The run planned in `docs/FINAL_RUN.md` (committed before any game was played), on frozen commit `7345e58`, trials
500-599, 100 games per condition, 800 games.  All 800 completed, none rerun; every game's settings and commit
matched its condition (checked by `cluster/analyze_final.py`).  Every number here is [measured] from that run
unless labelled otherwise.  Full output: `final_results/analysis/` (`report.md`, `comparisons.csv`,
`per_condition.csv`).

## The conditions

Every condition plays the same 100 starting towers (trial N is the same tower everywhere), so comparisons are
game against game on the same tower.

| | Condition | What changes | Blocks per game (mean, median) |
|---|---|---|---|
| A | random | picks a random block | 2.65, 2.5 |
| B | rule-based | picks by our rules | 6.14, 6 |
| C | rule-based + perception | B, but sees the tower through simulated cameras + ICP | 6.04, 6 |
| D | rule-based, varied friction | B, each block's friction 0.3-0.7 | 5.88, 6 |
| E | rule-based + probe | D, plus a 2 mm probe before each middle push, reject over 5.1 N | 5.89, 6 |
| F | random, varied friction | A, varied friction | 2.61, 3 |
| G | random + probe | F, plus the middle-push probe | 2.76, 3 |
| H | rule-based + probe on all blocks | E, plus an edge push-probe, reject over 3.9 N | 6.03, 6 |

The probe only matters when blocks differ, so the probe conditions use varied friction, and D and F are their
no-probe baselines: same towers, same friction, the probe is the only change.

## The comparisons

Difference = second minus first, paired by trial.  M1, M3 and M4 are Wilcoxon signed-rank tests, Holm-corrected
together; M2 is an equivalence test (TOST, margin +-0.5 blocks [user], set in advance); S1 is secondary, its own p.

| | Question | Result | Better / worse / same |
|---|---|---|---|
| M1 | Is rule-based better than random? (B vs A) | **+3.49** (95% CI +3.13 to +3.85), Holm p < 0.0001: **difference** | 95 / 1 / 4 |
| M2 | Does perception cost rule-based anything? (C vs B) | -0.10 (90% CI -0.26 to +0.06), inside +-0.5: **equivalent** | 6 / 13 / 81 |
| M3 | Does the middle probe help rule-based? (E vs D) | +0.01 (95% CI -0.16 to +0.18), Holm p 0.97: **no difference found** | 10 / 10 / 80 |
| M4 | Does the middle probe help random? (G vs F) | +0.15 (95% CI -0.03 to +0.33), p 0.051, Holm p 0.10: **no difference found** | 17 / 6 / 77 |
| S1 | Does probing every block help rule-based? (H vs D) | +0.15 (95% CI -0.20 to +0.50), p 0.52: **no difference found** (secondary) | 29 / 25 / 46 |

## What it says

- **The strategy matters most.**  Rule-based adds about 3.5 blocks a game over random and never fails early (0 of
  100 games at 1 block or fewer, against 22 for random).  Rule-based games end mostly on top-layer edge pulls (L5
  pull, 48) and placements on the new top layer (L0 place, 22); random games end on pulls all over the tower.
- **Perception costs nothing measurable.**  81 of 100 games are identical to B.  Median worst-block error 0.79 mm,
  largest 13.6 mm (cameras are noise-free, so this is what perfect cameras plus ICP give, D-27).
- **The force probe neither helps nor hurts.**  Rule-based rarely takes middles (11-14% of turns), so the probe
  fires rarely: 11 rejections in 100 games for E, and 80 games are identical to D.  Random takes more middles
  (21-27%) and was rejected 33 times; its +0.15 is the closest to a gain but is not significant after correction
  (say "a small improvement that was not significant").  Probing every block (H) rejects 161 blocks, changes
  which blocks are taken in half the games, but not the average, and takes 16.7 min per game against 11.0.
- **Why the probe does not help** (development runs, trials 0-99, D-28): the reading tracks each block's true
  friction (within-layer rank correlation +0.86 for middle pushes), but friction does not predict which turns
  fail (failed and placed middles 0.47 vs 0.49), and most middle failures happen after the block is out (11 of 15:
  regrab, place).  Sensing friction is not what limits this robot; handling is.
- The thresholds were fixed by a rule before the run (reject the 10% that resist most) and checked on trials 0-99
  only for cost (FINAL_RUN.md); they were not tuned to win.

## Replays

44 games chosen by the plan's rules after the results: best, worst and typical game per condition, and for each
comparison the trial where the second condition gained most and where it lost most.  All 44 reran identically
(`cluster/replays.py verify`).  Manifest `final_results/replays/replays.csv`; for the video,
`final_results/katie_replays/` (readable manifest, recreate commands, README; the videos are shared separately as
`katie_replays.zip`).

## Where the data is

`final_results/` in the repo, one folder per game: `<letter>/<strategy>_trial<NNN>/` with `trials.csv`,
`turns.csv`, `steps.csv`, `probes.csv`, `overrides.txt`, and `<letter>/code.txt` (the commit).  Redo the analysis
with `python cluster/analyze_final.py final_results --commit 7345e58` (in the container).

Committed: those text files, `analysis/`, `replays/replays.csv` and `selection.csv`, and the bundle's text files.
Not committed (too big; on ORCD only): the replay videos (`*.html`, 1.5 GB), each game's `assets/` (block files,
made again from the seed) and `katie_replays.zip`.  Do not `git add final_results` as a whole.
