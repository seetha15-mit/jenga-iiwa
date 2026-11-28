# Final run: the analysis plan (fixed before the run)

DRAFT - to be agreed by the team, then committed before any final game is run.  Git's date on that commit is the
record that the plan came first.  Nothing in it changes after results come in; anything added later is marked
"added after the run" and is not one of the main results.

## What is run

- **Code**: one commit of `main` with the force probe merged (D-28), frozen in a git worktree for the whole run.
  `python run_checks.py` passes on it first (all checks, with the static-check fix).
- **Container**: `jenga.sif` built from `cluster/jenga.def`, Python packages pinned by `cluster/requirements.lock`.
- **Scripts**: the conditions are defined once in `cluster/final_conditions.py`; `cluster/final_run.sh` submits them
  (refuses a working copy with uncommitted changes), `cluster/analyze_final.py` does every test and table below and
  the validity checks, `cluster/replays.py` chooses, reruns and checks the replays and writes their manifest.
- **Robot and tower**: the defaults on that commit: turntable base (D-02), 6 starting layers, hydroelastic contact,
  mixed method (D-26: push middles, end-pinch edges), push only up to layer 3, real mover (never the fake one).
- **Trials 500-599**, never used before (0-99 development and the probe sweep, 100-199 D-23, 200-299 O-8, 300-399
  D-26 and the turntable check, 400-499 D-27).  Trial i uses seed 1000 + i.  Every condition plays the same 100
  trials, so comparisons are paired.  The first block of each game is random (course rule, R-03) and comes from the
  trial's seed, so paired games start with the same block (unless the probe rejects it and a new one is drawn).
- **Logging**: development log level 1 on, replays off.  Logging only records: reruns at log level 2 and with
  replays on matched their level-1 originals block for block.  Replays are recorded afterwards for chosen games
  (see Replays).

| | Condition | Strategy | Sensing | Friction |
|---|---|---|---|---|
| A | Random | `random` | simulator truth | equal (`global`, 0.5) |
| B | Rule-based | `rule_based` | simulator truth | equal |
| C | Rule-based + perception | `rule_based` | cameras (`PERCEPTION=True`, D-27) | equal |
| D | Rule-based, varied friction | `rule_based` | simulator truth | varied (`random`, 0.3-0.7) |
| E | Rule-based + force probe | `rule_based` | truth + wrist-force probe on middle pushes (`PROBE_ENABLED=True`, D-28) | varied |
| F | Random, varied friction | `random` | simulator truth | varied |
| G | Random + force probe | `random` | truth + wrist-force probe on middle pushes | varied |
| H | Rule-based + force probe on all blocks | `rule_based` | truth + probe on middle pushes and edges (`PROBE_EDGE_PUSH=True`) | varied |

D and F are the baselines for E and G: varied friction makes different towers, so the probe is judged against the
same strategy on the same varied-friction towers.  The probe works with any strategy (a rejected block is dropped
and the strategy picks again; random draws again), so it is tested with both: random takes risky blocks more often,
so it shows how much the probe can catch, and rule-based shows what it adds on top of the rules.

**The probe, as fixed on development trials 0-99 (varied friction; `cluster/probe_sweep.sh`, D-28):** middle
pushes only (`PROBE_END_PULLS=False`), at the push's own speed (2 mm in `PROBE_SECONDS=0.1`), rejecting a push
over `PROBE_MAX_PUSH_N=5.1` N.  Why (all paired against probe off on the same 100 towers):
- A probe leg slower than the motion dragged the layer above: 2 mm in 1 s cost 0.71 blocks per game (p = 0.004);
  at the push's own speed pushes moved the other blocks no more than unprobed ones.
- Edges are not probed: any 2 mm move of a gripped edge block before end-pinch's 1 m/s pull tipped marginal pulls
  (gripped probe -0.38, p = 0.017; 0.5 mm -0.26).  E and G probe middle pushes only.  H (rule-based only, secondary)
  also probes edges, with a fingertip push along the edge block from an end face, backed out before the normal
  end-pinch pull (`PROBE_EDGE_PUSH`): -0.11 blocks per game measure-only (p = 0.51, 78 of 100 identical).
- Push-only probing measure-only: -0.02 blocks per game (p = 0.97), 79 of 100 games identical.
- Thresholds by one rule fixed before checking them: reject the 10% of blocks that resist most.  Middles 5.1 N (90th
  percentile of push-only's 105 push readings, 5.12 N); H's edges `PROBE_MAX_PULL_N=3.9` N (90th percentile of the
  edge push-probe's 620 edge readings, 3.93 N).  Checked on trials 0-99 against probe off and kept: middles at
  5.1 N -0.01 blocks per game (p = 0.88, 80 of 100 identical, 14 of 107 pushes rejected); all blocks at 5.1 / 3.9 N
  -0.08 (p = 0.68, 46 identical; 145 of 744 edge and 19 of 141 middle probes rejected).
- The reading tracks the block's true friction (within-layer rank correlation +0.86 for pushes) but friction does
  not predict failure (failed and placed middles 0.47 vs 0.49; most middle failures happen in the regrab and place,
  after the block is out), so the probe is not expected to help much.

## Main comparisons

The measure is **blocks per game**: blocks extracted and replaced before the tower collapses (R-03).

| | Question | Test | Claim if |
|---|---|---|---|
| M1 | Is rule-based better than random? (B vs A) | Wilcoxon signed-rank on paired blocks per game, two-sided | p < 0.05 after Holm |
| M2 | Does perception lose anything against the truth? (C vs B) | Equivalence (two one-sided t-tests) on the paired difference, margin +-0.5 blocks per game | both one-sided p < 0.05 (the 90% CI lies inside +-0.5) |
| M3 | Does the force probe help rule-based on varied-friction towers? (E vs D) | Wilcoxon signed-rank, two-sided | p < 0.05 after Holm |
| M4 | Does the force probe help random on varied-friction towers? (G vs F) | Wilcoxon signed-rank, two-sided | p < 0.05 after Holm |

- **Secondary, S1**: does probing all blocks change rule-based on varied-friction towers? (H vs D), Wilcoxon,
  two-sided, reported with its own p value (not in the Holm family).
- Holm's correction is applied across M1, M3 and M4 together.  M2 is an equivalence test and is reported on its
  own (the two one-sided tests already control its error rate).
- Every comparison is also reported as the mean paired difference with its 95% CI, and the counts of games
  better / worse / the same.  A non-significant M1, M3 or M4 is reported as "no difference found", not as
  "the same"; only M2 makes a "the same" claim.
- Why +-0.5 blocks: about 8% of a 6-block game, and what 100 paired games can resolve (paired SD about 2 blocks,
  so the 90% CI is about +-0.33 when there is no true difference; D-26).  Fixed here, before the run.

## Also reported (descriptive, no tests)

- Blocks-per-game distribution per condition; games with 6 or more blocks; the longest game.
- **Early failures** (games with 1 block or fewer, as in D-23): counts per condition, and for each pair the games
  where only one of the two failed early.  Not tested: paired games share their first block, so without the probe
  an early-failure difference can only come from the second turn.
- How games end: the step (pull, push, pedestal, regrab, place), the method (end-pinch or push) and the layer,
  as in D-26 and `docs/IMPROVEMENTS.md`.
- Share of turns that were middles.
- C: perception error per turn (`perception_max_pos_mm`, `perception_max_height_mm`), blocks never seen, turns
  over the 0.875 mm finger margin.
- E, G and H: probes per game (H: edges and middles apart), rejections, how often the turn fell back to the
  least-resisting block, resistance (N) of accepted and rejected blocks (`probes.csv`), games where the probe itself
  knocked the tower.
- Wall time per game per condition.

## Validity rules

- A game counts when its `trials.csv` row exists.  A game that did not finish (time limit, crash, out of memory) is
  rerun once with the same code and trial (games repeat exactly from their seed).  If it fails again it is left out
  of every comparison that uses that trial, and the number left out and why are reported.
- Each trial folder's `overrides.txt` must show exactly that condition's settings, and `code.txt` the frozen commit;
  the analysis script checks both and stops on any mismatch.
- **Probe-off check**: 10 rule-based games on trials 300-309 (equal friction, probe off) match the latest games on
  the same robot (`runs_scratch/turntable_check/turntable_rb`, before the force merge) block for block: the same
  block, layer, slot, outcome and collapse in all 73 turns, the same end in all 10 games (run on 3fd2c4e; later
  commits before the freeze change only the run's scripts, not the game code).  So the force merge did not change
  games with the probe off, and earlier results quoted in the paper (D-26, D-27) still describe the final code.
- No forced first block (`DIAG_FIRST_BLOCK` unset), no fake mover, no runs from a working copy.

## Replays (after the results)

Games are chosen by these rules, then rerun on the frozen commit with replays on.  A rerun counts only if its
`trials.csv` row matches the original (same blocks per game, same end); reruns are kept in their own folder.

- **Per condition, three games**: the best (most blocks), the worst (fewest blocks) and a typical one (blocks
  closest to the condition's median).  Ties go to the lowest trial number.
- **Per comparison (M1-M4, S1), two pairs**: the same trial under both conditions, the trial where the second
  condition gained most and the one where it lost most.  Ties go to the lowest trial number.
- Best, worst and pair games are chosen to illustrate, and are labelled as such ("longest game", "largest
  difference"); the typical game is the one that shows what a condition usually does.
- **Manifest**: one row per replay in `replays.csv` next to them: why it was chosen, condition, strategy, trial,
  seed, every setting (`overrides.txt`), commit, container (`cluster/requirements.lock`), the command that made it,
  blocks per game, and the replay file.  Games repeat exactly only on the same platform and container (marginal
  grasps went opposite ways on a Mac and the cluster with the same code, O-5): replays are made on the cluster and
  the replay files are shared with the manifest, not remade elsewhere.

## Not in the final run (and why)

- End-only rule-based: already compared with mixed on the current robot (D-26: 6.18 vs 6.19 on fresh trials;
  turntable check D-02).
- Probing edges with a gripped pull, and random with the all-blocks probe: development results only (above).
- Random with perception: perception matched the truth for rule-based (D-27); the paper's question is whether it
  costs the rule-based result anything.
- Turntable vs rolling base: no measurable difference (D-02); every condition uses the turntable.
- Noisy cameras: the simulated cameras are noise-free, so C shows what perfect cameras plus ICP achieve, not a real
  camera (D-27).

## Batches

Eight conditions are 800 jobs, over the cluster's 500-job submit limit, so they go in two batches from the same
frozen commit: A-D, then E-H when A-D are done.  About 10 min per game for A, B, D and F; C, E, G and H are slower
(cameras; probes; H's extra base move per edge).  While E-H run, M1 and M2 (A, B, C only) are analysed and their
replays made; nothing about the conditions, thresholds or comparisons changes between batches.

Settled: varied friction (0.3-0.7) runs normally with rule-based (6.29 blocks per game) and random (2.65) on
trials 0-99.
