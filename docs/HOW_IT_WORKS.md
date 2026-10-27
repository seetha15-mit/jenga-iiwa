# How it works (one page)

## The world
One table (top at z = 0), a **platform** under the tower, the **tower** (blocks are free bodies), and the **robot** on a rolling base.
The tower never moves. The robot rolls between **stations** (positions around the tower, facing it) instead. There is ONE simulator for a
whole trial, so the replay is one continuous recording.

The plant is built by hand, as in the psets (`iiwa_1.py`, `mobile_base_ik.py`): parse the arm, the gripper and our boxes, weld them, and
drive every actuated joint (arm, fingers, rolling base) with PD control built into the plant. The arm is gravity-compensated (gravity is
off for its links only), so the gripper keeps its weight.

Frames: **W** world (tower centre at the origin). **R** robot base frame (x forward from the arm's mounting flange, z up). **G** gripper
frame (x = finger axis, y = reach axis toward the fingertips, z = the third axis). Your steps work in **R**: they never need to know
which station the robot is at.

## One turn
```
harness picks block (first pick of a trial is random; later ones come from the strategy)
   |  candidates = legal pulls (tower rules) that the method can physically take (end-pinch: edge blocks)
   v
mover.do_turn
   1. choose station -> executor rolls the base there (arm tucked)        [shared]
   2. plan_pull(X_RB)   -> waypoints -> executor plays them               [YOU WRITE plan_pull]
   3. choose place station (maybe a different one) -> base rolls          [shared; block stays in the hand]
   4. plan_place(X_RT)  -> waypoints -> executor plays them               [YOU WRITE plan_place]
   v
harness (evaluation side, true poses): did the block arrive? (replaced / failed pull / dropped)
   and did the tower collapse? -> log a CSV row, end the trial on collapse
```

## Who knows what
- **Shared, knows the tower only:** `tower.py` (where blocks are, which are legal, where a block goes next).
- **Per method, knows the hand:** `methods/end_pinch.py` (which blocks the hand can take, which station, the hand poses).
- **Evaluation, knows the truth:** `evaluation/` (true block poses, the collapse verdict). Strategies never see any of it.
- **Hidden from strategies:** per-block friction and exact contact forces.

## Why only some stations work (derived, untested in simulation)
The hand holds a block by its ends, so the block's long axis is always across the reach direction. Layers alternate direction, so at a
station the robot can only work on every second layer; the other layers are reached from a station 90 degrees round. A level hand's body
(50 mm tall, about 72 mm behind the pads) would hit the layer below when placing into the FAR slot of a layer, so that slot is filled from
the opposite station, where it is the near slot.

## Motions
A plan is a list of `Waypoint(X_RG, opening)`. The executor reaches the first waypoint with a joint-space move (safe for big moves) and the
rest in straight lines (precise). It warns if the hand ends far from a target or a joint ends near its limit. **Arm collisions are not
checked** (the arm model has no collision geometry): watch the replay.

## Files written per trial
`turns.csv` (one row per turn), `trials.csv` (one row per trial), `trial_XX_replay.html` (open in a browser), optionally the raw wrist
force. Fake-mover runs go to `runs_fake/` and are tagged `fake`; the results script refuses them.
