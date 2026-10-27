# Jenga-playing iiwa in Drake (6.4212 final project)

A simulated KUKA iiwa 7 with a Schunk WSG-50 gripper pulls a block out of a Jenga tower and puts it on top. We compare
block-selection strategies by how many blocks are extracted and replaced before the tower collapses.

## Three commands
```
python run_checks.py                      # checks everything, writes checks_report.txt 
python run_experiment.py --mover fake     # runs trials with the placeholder mover (results INVALID, saved in runs_fake/)
python make_results.py                    # results table from REAL runs only (refuses fake ones)
```
Setup: `docs/SETUP.md`.  How it works: `docs/HOW_IT_WORKS.md`.  What each of you writes: `docs/FOR_THE_TEAM.md`.
Why things are the way they are: `docs/DECISIONS.md`.

## Where things are
| Where | What |
|---|---|
| `config.py` | EVERY tunable value, each tagged with where the number came from |
| `jenga/tower.py`, `jenga/geometry.py`, `jenga/base.py` | tower state and rules, sizes, base stations and paths (plain Python) |
| `jenga/scene.py`, `jenga/executor.py`, `jenga/controller.py`, `jenga/kinematics.py` | the Drake side: one persistent simulator, motions, IK |
| `jenga/methods/` | extraction methods. `end_pinch.py` has the team's `plan_pull` and `plan_place` |
| `jenga/strategies/` | block selection: `random_baseline.py` works, the others are team stubs |
| `jenga/evaluation/` | collapse detector, turn outcomes, the harness, the results table (never visible to strategies) |
| `jenga/mover.py` | the fake mover (placeholder) and the real one (calls the team's steps) |
| `runs/`, `runs_fake/` | real and fake outputs: CSVs and `trial_XX_replay.html` (open in a browser to watch at normal speed) |
| `tests/` | tests for everything that does not need Drake |
| `utils/` | course code (psets), with marked edits |
