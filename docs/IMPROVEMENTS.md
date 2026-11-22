# Possible improvements (not decided)

Found while analysing the rule-based runs (Nov 2026), left for later.  Neither is a decision: when one is taken up,
it goes through an A/B and then into DECISIONS.md.  Numbers come from three 100-game runs with the development log on
(`DIAG_LOG_LEVEL=1`): end-only V2 + column filter on trials 0-99 and 300-399, and mixed with the full-above filter on
trials 0-99.  Together these two problems are where almost every game now ends, for end-only and mixed alike: no
game gets past 8 blocks.

## 1. First block of layer 8 slides off a leaning tower (placement)

**What happens** [measured]: placing the first block of layer 8 (slot 0) collapsed the tower in 24 of 78 unique cases
(31%); it ended 11-30 of every 100 games.  Placements into layers 6 and 7 almost never fail.  Before release, failing
and successful placements look the same (arm pose, block tilt, hand height, layer 7 placed equally well).  In the
1 s hold after the fingers open, the released block drops about 17 mm, tilts about 28 deg, slides about 48 mm
sideways and shoves a layer-7 block.  Confirmed in two replays of the same situation, one failing and one not
(end-only trials 302 and 305, turn 6).

**Likely reason** [measured correlation, interpretation]: these placements come right after a late layer-0 end-block
pull.  That pull leaves layer 0 on its middle and one end, and the tower leans toward the gap:

| Tower tilt after that turn's pull | Placements | Failed |
|---|---|---|
| under 0.6 deg | 41 | 5 (12%) |
| 0.6 deg or more | 37 | 19 (51%) |

A new layer always starts at slot 0 (`tower.placement_slots` returns slots 0, 1, 2 in order).  Layers 0 and 8 run
the same direction, so layer 8 slot 0 sits right above layer 0 slot 0: after a slot-0 pull the new block is set
down over the missing support.  Pulled layer 0 slot 0: 17 of 34 failed; pulled slot 2 (gap on the other side): 7 of
38.  Five failures had little tilt, so something else plays a part too.

**Possible solution**: start a new layer with the edge over the better-supported side, i.e. the edge whose column of
same-direction layers below is complete (as `rule_based.column_intact`); on a tie keep slot 0.  A few lines in
`tower.placement_slots` (shared code).  Expected gain about +0.1 to +0.3 blocks per game [estimate].  It changes
every game from layer 8 on, so it needs its own A/B (end-only, trials 0-99) and new baselines for both strategies.
Other options: a gentler set-down (the hand ends about 5 mm below its target in every placement), or the middle
first (D-08: the far edge then hit the middle).

## 2. Layer-5 end-block pulls always end the game (pull)

**What happens** [measured]: all 121 layer-5 end-block pulls in the three runs ended the game, whether the block's
column was hollow or intact.  The tower starts to move on the pull-out leg (pull waypoint 4), and the top moves
first: the part-built layer 8 in 97 cases, layer 7 in 24.  The hand does not hit anything on the way in.

**Likely reason** [interpretation]: by then layer 5 has only two complete layers above it (6 and 7) plus a partial
layer 8, too little weight to hold the top still against the fast end-pinch pull-out (0.1 s, D-23).  Pulls from
layer 4, with three or more layers above, end the game about 16% of the time.  The push method had the same problem
(O-5: it takes a block only with two complete layers above, `PUSH_MIN_LAYERS_ABOVE`, and pulls slowly while the block is inside the tower, `PULL_INSIDE_SPEED_MS`).

**Possible solution**: a slower pull-out high in the tower, as `PULL_INSIDE_SPEED_MS` does for push.  That is in
`end_pinch.plan_pull` (team-owned code).  A selection rule would not help much: end-only only pulls from layer 5 when
nothing else is left.  It would need an end-only A/B like any method change.

## How these were found
The step log (`steps.csv`): the step and waypoint where the worst-moved block first passes 5 mm, which layer it is in,
the worst tilt after each step, and the block's offset in the hand (`in_hand_*`).  For the placement, the tower state
was rebuilt from `turns.csv` and repeated tower states (same seed, same picks in another run) counted once.
