# What you write, and how to test it

Everything marked `CODE TO BE WRITTEN` is yours. You must be able to explain and justify every robotics choice in it (the course quizzes
each of you on the whole project), and say in the commit message if an AI tool helped.

## 1. The pull and the place (the first job: get ONE block out of a 3-layer tower)
File: `jenga/methods/end_pinch.py`. Two functions, each returns a list of `Waypoint(X_RG, opening)` (`jenga/waypoint.py`):

| Function | Receives | Returns |
|---|---|---|
| `plan_pull(X_RB, cfg)` | pose of the block in the robot frame R (origin: centre of the block's bottom face, x: long axis) | waypoints: approach, pinch, pull out sideways, finish holding the block clear |
| `plan_place(X_RT, cfg)` | the target pose for the block on top, in R | waypoints: carry over the tower, lower, release, back away |

Units: metres, radians. `opening` is the finger command in metres; the pad gap is the opening + 5 mm. `None` means fully open (110 mm).
Useful: `jenga.geometry.block_dims(cfg)`, `jaw_margin_per_side_m`, `finger_height_margin_per_side_m`, `cfg.PAD_CENTRE_FROM_G_M`
(pads are 0.108 m from G along G y), `cfg.FINGER_SQUEEZE_M`, `cfg.WAYPOINT_SPEED_MS`.

Things you decide: hand orientation (level: G y horizontal, G x along the block's length), the approach path, pad height, squeeze, pull
distance and speed. Things to watch (all derived, untested in simulation): the hand must be within about **0.9 mm of the right height**
or the pads touch the layers above and below; arm collisions are not checked; the first waypoint of a plan is reached by a joint-space move,
so make it a pose that is clear of the tower. If IK fails for a pose, try the same grasp with the hand rolled 180 degrees about G y.

**Test it:** set `MOVER = "real"` (or `--mover real`), then `python run_experiment.py --mover real --trials 1 --turns 1`, then open the replay.
The CSV says what happened (`outcome`, `mover_error`).

## 2. The strategies
`jenga/strategies/`: `rule_based.py`, `perception.py`, `force_guided.py`. One function each: `select_block(tower, candidates, cfg, rng, sensors=None)`
returns one block id from `candidates`. Use the `rng` passed in for any randomness. `random_baseline.py` is a working example. Hidden from
strategies: per-block friction and exact contact forces. The first block of every trial is random, whatever the strategy (course rule).
Force and perception need sensors: read `jenga/sensing.py` (notes only; nobody has decided the interface).

## 3. The advanced collapse detector
`jenga/evaluation/collapse.py`, function `advanced(truth, cfg)`; set `COLLAPSE_DETECTOR = "advanced"`. It receives true block poses before and
after the turn and the tower bookkeeping, and returns `Verdict(stop, reason)`. The basic one only compares positions to a threshold.

## 4. Arm clearance (a TODO nobody has decided)
`jenga/clearance.py`. The arm can pass through the table, platform and tower without the simulator noticing. Until this exists, results
carry `arm_collisions_unchecked = True`. Options: an iiwa model with collision geometry, geometric proxies on each planned path, or just watching replays.

## Values to tune (all in `config.py`)
`PLATFORM_HEIGHT_M` and `STATION_STANDOFF_M` are starting guesses; `run_checks.py` has a reachability check that says whether every layer
can be reached. `CONTACT_MODEL`: try `"point"` (`python run_checks.py --compare-contact`) and switch if it holds the tower still and is faster.
