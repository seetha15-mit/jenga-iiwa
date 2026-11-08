# DECISIONS

States only what is true NOW.  Change an entry in place; git keeps the history.  Each entry: what, one-sentence why, source.
Number sources: `user`, `course`, `measured`, `derived`, `estimate` (estimates get verified or replaced).  Names in `code` show what a change touches.

## Requirements (change only with the owner's say-so)
| id | Requirement | Source |
|---|---|---|
| R-01 | Simulated iiwa + WSG-50 plays Jenga in Drake: extract a block, replace it on top | course |
| R-02 | Compare >= 2 selection approaches (random, rule-based, perception, force-guided); finish the more promising of perception/force | course |
| R-03 | Metric: blocks extracted and replaced before collapse; minimum 4; >= 10 trials per approach; first block random | course |
| R-04 | Start with 3 layers (12 eventually) | course (TA) |
| R-05 | Per-block friction and exact contact forces are hidden from every strategy; force strategy reads the wrist force only | course (TA) + user |
| R-06 | End-pinch default: hand from the side, jaw on the two END faces of an edge block, slide out sideways, place on top as it is | user |
| R-07 | A push method (maybe with a tool) stays possible later; only room in the architecture | user |
| R-08 | One uniform scale factor; flush layers | user |
| R-09 | Tower on a raised platform | user |
| R-10 | No assumed hand tilt (level hand) | user |
| R-11 | Simulations watched only afterwards, as a normal-speed replay | user |
| R-12 | The team writes the pull/place steps, the strategies, the advanced collapse detector; AI use must be citable | user + course |
| R-13 | All tunable values in one place; stubs with docstrings; docs for non-programmers | user |

## Facts the design respects
F-01 The arm model has no collision geometry; only gripper, blocks, table collide (measured, from the model file name; not yet confirmed in Drake).
F-02 Gripper frame G: x finger axis, y reach axis. Hand body 146 x 72.5 x 50 mm; fingers 20 mm tall; pads 20 x 20 x 3 mm; pad gap = command + 5 mm; fingertips 0.118 m from G (measured, SDF).
F-03 The wrist centre is on the reach axis 0.171 m behind G, so a level hand puts the wrist at pad height (derived; matched the home pose to 1 mm).
F-04 About 0.1x real time with hydroelastic contact, 9 blocks plus gripper, one machine (measured; no real turn measured yet).
F-05 The pset Cartesian controller ignores joint limits (measured). F-06 Wrist force reads in the gripper frame and includes the hand's weight, 10.7 N at rest (measured).
F-07 Drake 1.57.0, Python 3.13 on WSL2 (manipulation 2026.9.17) and on macOS arm64 (manipulation 2026.10.17); full `run_checks.py` passes on macOS (measured).
F-08 The hardware station's iiwa driver aborts (`inverse_dynamics_controller.cc: num_positions == dim`) when the arm is welded to a body that has joints; a base model alone and an arm welded to a fixed named frame build fine (measured, `diagnose_base.py` run).

## Decisions
| id | Decision | Why | Source |
|---|---|---|---|
| D-01 | I build the shared infrastructure and a fake mover; the team writes the real pull and place (`methods/end_pinch.py`: `plan_pull`, `plan_place`). The baseline is "done" when those are plugged in and random plays real trials | the team must own the manipulation code | user |
| D-02 | Rolling base (three joints x, y, yaw) with a list of stations (`STATION_ANGLES_DEG`); the team's steps get targets in the robot frame only; replace `choose_station` and `base.base_path` to move to a free-moving base | reaches every face without touching the tower | user |
| D-03 | One persistent simulator per trial (`scene.Sim`), plant built by hand in the pset style (`AddMultibodyPlantSceneGraph` + `Parser`, no hardware station) | no tower reset or rebuild; the station cannot drive an arm on a rolling base (F-08) | user |
| D-04 | Plan-then-execute: the team returns waypoint lists, `executor.run_plan` plays them (first waypoint by joint-space move, the rest straight lines). A leg takes the arriving waypoint's `seconds` if set, else distance / `WAYPOINT_SPEED_MS` (at least 0.1 s); a waypoint's opening is commanded from the start of the next leg | learns the pset pattern; no Drake systems for the team; per-leg times let a pull be slow and the fingers finish closing before the hand moves | user |
| D-05 | `PLATFORM_HEIGHT_M` 0.10 is a starting value for the team to sweep. `STATION_STANDOFF_M` 0.95 and `BASE_TRAVEL_RADIUS_M` 1.2: Drake IK passed 0.90, 0.95 and 1.00 m (worst joint margin 16, 26 and 31 deg over all layers and poses) and failed 0.80 and 0.85 m | a level hand needs the tower far out (wrist 0.171 m behind the pads) | user / measured (IK only, no collisions) |
| D-06 | Block scale 1.45 = 108.75 x 36.25 x 21.75 mm, gripper command 110 mm. Margins per side: jaw 3.1 mm, finger height 0.875 mm | the finger-height margin is the tight one | user / derived |
| D-07 | `EXTRACTION_METHOD` switch; `methods/push.py` is the push method (D-21, D-22). Shared: tower state, legal pulls, collapse, logging. Per method: reach rules, plan_pull, plan_place | room for push without designing it | user |
| D-08 | Place-on-top is split: `tower.placement_slots/target_pose` (shared) and the method's own hand pose. End-pinch fills the far slot from the opposite station | the level hand body (50 mm tall, ~72 mm behind the pads) hits the layer below at the far slot, about 20 mm over, 14 mm deep | derived |
| D-09 | Legal pulls: any block except the highest complete layer and the incomplete layer above it; bottom layer allowed. End-pinch adds: edge blocks only | classic Jenga; a middle block cannot slide out | user |
| D-10 | Hydroelastic contact by default; `CONTACT_MODEL`, `CONTACT_APPROXIMATION` ("lagged"), `PLANT_TIME_STEP` (1 ms) in config | with "sap" a pull of block 0 dragged layer 1 out with it (70 mm of a 100 mm pull) although layer 1 rested only 0.42 N on block 0 and 3.11 N on its neighbours; Drake documents SAP "gliding" in sliding contact. With "lagged" the same plan moved no other block more than 0.2 mm. Lagged: the tower moves 1.04 mm in 3 s, 0.11x real time, full checks about 15-20% slower than sap. Point contact: 0.32x real time but 17 mm in 3 s (limit 2 mm), so it is not used | measured |
| D-11 | Replay: Meshcat recording of the persistent simulator saved as `trial_XX_replay.html`; CSV per turn and per trial; optional raw wrist-force trace | least code; fallback is our own pose log. Each trial starts with `START_SETTLE_S` of standing, so the replay opens on the intact tower | user |
| D-12 | Collapse detector is evaluation-side: reads truth, returns continue/stop plus a reason; the basic one uses poses and `COLLAPSE_DISPLACEMENT_M` | strategies must never see it | user |
| D-13 | Fake-mover runs are tagged `mover=fake`, written to `runs_fake/`, and `make_results.py` refuses them | no faked run in the results | user |
| D-14 | Turn outcomes: replaced, failed_pull, dropped, collapse. A failed turn is logged and the strategy picks again; `MAX_CONSECUTIVE_FAILURES` ends the trial | one slip should not end every trial | user |
| D-15 | Trial i uses seed `BASE_SEED + i`: the first block (random, same for every strategy) and the friction draw each have their own stream | fair comparison, reproducible | user |
| D-16 | Starting physics: gap 0.5 mm, density 700 kg/m^3, friction 0.5 | something must make the tower stand | estimate |
| D-17 | One `config.py`, every line tagged with its source; `python run_checks.py` writes one `checks_report.txt` | easy for non-programmers | user |
| D-18 | Docs in `docs/`; `CLAUDE.md` has ground rules for AI tools; pset-style plain code with docstrings | course and team level | user |
| D-19 | Arm, fingers and base are PD-controlled joints of the plant (`ARM_KP/KD`, `FINGER_KP/KD`, `BASE_KP/KD`); gravity is off for the ARM links only (an ideal gravity-compensated arm), so the hand keeps its weight in the wrist force | the psets' own controllers are not in the project files and ran at a 100 us step (too slow for contact); the plant's PD joints appear in no pset; the executor check passes, but no grasp has been tried | measured (no-contact motions only) |
| D-20 | The project has its own virtual environment (`.venv` in the repo, Drake pinned to 1.57.0), made with the course's uv steps (`docs/SETUP.md`) | package versions stay fixed and the pset setup is untouched | user |
| D-21 | Push stations (`base.push_stations`): the robot faces a block's short END face (approach perpendicular to the layer's across axis) and pushes along its long axis; any slot, middle included, and either end, so two stations per block. A push turn is five steps (`RealMover`, when the method has `plan_push`): `plan_push` at the nearer end, `plan_pull` from the opposite station where the block sticks out (pose read again after the push), `plan_ped_place` (set it on a D-22 pedestal) and `plan_regrab` there (pose read again; a separate plan so it starts with a joint move, which clears the feedforward controller's drift: as one motion the hand was 11-13 mm off at the grasp and dropped the block, measured), `plan_place` | a push slides the block out the far side, past no neighbour; the protruding end is easiest to reach from the far side | user |
| D-22 | Regrab pedestals (push method): four welded posts (`base.pedestal_poses`), one on each diagonal between two stations, `PEDESTAL_DISTANCE_M` 0.27 from the tower centre, top `PEDESTAL_HEIGHT_M` 0.07, square top `PEDESTAL_TOP_M` 0.03 turned so a side faces the tower. The pulled block is set on one with its long axis pointing at the tower, then picked up by its ends with a level hand | keeps the level hand (R-10) and the 0.95 m standoff: a level hand cannot pick a block off the table (F-02, F-03) and a top-down place is out of reach at 0.95 m. IK only, from the adjacent station: put-down 33 deg and end-pinch regrab 29 deg worst joint margin. Top 16 mm under the lowest hand body near the tower (layer 0, 0.086 m), corner 7 mm outside the push hand's sideways reach (0.163 m off the axis), narrower than a block so the fingers pass beside it | user (idea) / derived (not collision-checked, O-1) |

## Open items (move to Decisions when decided)
O-1 **Arm collisions are not checked** (TODO, team). Hooks: `clearance.check_motion`, CSV column `arm_collisions_unchecked`.
O-2 **Sensing**: interface, weight compensation, cameras are for the owner of those strategies to decide (`sensing.py` has notes only).
O-3 Point contact is 3x faster but the standing tower drifts 17 mm in 3 s (measured, 1 ms step, SAP). If speed becomes the bottleneck the team can try a smaller `PLANT_TIME_STEP` with `CONTACT_MODEL = "point"` (`run_checks.py --compare-contact`), then re-check the tower and a real pull. Do not go back to "sap" for pulls (D-10).
O-4 Platform height and standoff sweep; the reachability check reports how far the default is from working.
O-5 Push method details beyond its stations (D-21): tool, tolerances, how far to push, how the pushed-out block is grasped, regrabbed (on a D-22 pedestal: which one, and the waypoints) and placed (`push.plan_push`, `plan_pull`, `plan_ped_place`, `plan_regrab`, `plan_place`, `can_place`, `choose_place_station`). Status (measured, block 1): push 0.4 L moves the tower 1.5 mm; after a real push the pull moves it at most 5.3 mm with a squeeze of W - 1.75 x FINGER_SQUEEZE_M (194 mm at W - 1 x, 8.0 mm at W - 1.5 x); a full turn through `run_experiment.py` (trial 0, block 1) ended "replaced". The pull's weak point: block 1 tips about 0.4 deg in the pads as it starts to slide (traced; the hand stays level), which loads the layer-1 block above; a firmer squeeze shortens the drag, an open-loop hand tilt made it worse (+-0.4 and -0.13 deg tried). Only block 1 is tuned. Pull only with two or more layers on top (block 4 under the top layer dragged layer 2). O-6 Placement beyond 6 layers (carry pose and hand-body clearance over a taller tower).
O-7 Finger-height margin 0.875 mm: check in Drake that a level hand is accurate enough (first thing for the pull).

## Checked in Drake, and what is still unchecked
Passed in full `run_checks.py` (measured, 2026-10-29, macOS): the hand-built plant builds with SAP and hydroelastic contact; the tower
stands (1.05 mm in 3 s); the pad gap read from the model (115 mm); the robot frame agrees with the full scene (0.00 mm); the wrist-force
port (10.7 N at rest, so the hand keeps its weight); the PD joints follow commands in a running simulator (four executor motions); the
rolling base visits stations without disturbing the tower; a 2-turn fake trial writes tagged files and a replay.
Still unchecked: any real grasp or pull (O-7), arm collisions (O-1), a Meshcat recording of a long trial, the time of a real turn (F-04).
