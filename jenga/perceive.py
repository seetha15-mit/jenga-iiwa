"""Perception in games (stage 3): with config.PERCEPTION on, the tower bookkeeping comes from the cameras
(sensing.estimate_block_poses, the team's) instead of the simulator's truth.  Plumbing only; reads no truth.

The harness calls observe() at the start of every turn.  What this file decides is ICP's starting guess per block:
  * a block still in the slot it was last seen in: its LAST PERCEIVED pose;
  * otherwise (a trial's start, or the block was just placed in a new slot): its slot's pose at the SETTLED height,
    platform + layer * H.  Blocks rest on each other, so a settled tower has no gaps; built-height guesses left
    layers 1-4 0.6-1.5 mm high, settled-height ones within 0.31 mm (scratch/check_perception.py, fresh tower).
A block no camera sees keeps its guess.
"""

import numpy as np

from jenga import base
from jenga import geometry as geo
from jenga import sensing


def settled_slot_pose(tower, layer: int, slot: int, cfg) -> np.ndarray:
    """The slot's pose with its height at platform + layer * H (no gaps: blocks rest on each other)."""
    X = np.array(tower.target_pose(layer, slot), dtype=float)
    X[2, 3] = cfg.PLATFORM_HEIGHT_M + layer * geo.block_dims(cfg)[2]
    return X


def guesses(tower, cfg, last: dict) -> dict:
    """ICP starting pose per block in the tower.  last: block id -> ((layer, slot), 4x4) from the previous observe
    (empty at a trial's start).  Blocks not in a slot (loose) are not guessed."""
    out = {}
    for (layer, slot), b in tower.slot_map.items():
        prev = last.get(b)
        out[b] = prev[1] if prev is not None and prev[0] == (layer, slot) else settled_slot_pose(tower, layer, slot, cfg)
    return out


def observe(world, tower, cfg, last: dict):
    """Take pictures and estimate the tower.  Returns (poses: block id -> 4x4 for tower.update_poses,
    new last: block id -> ((layer, slot), 4x4) for the next call, unseen: sorted ids no camera found)."""
    g = guesses(tower, cfg, last)
    est = sensing.estimate_block_poses(world.camera_images(), cfg, g)
    poses = {b: np.asarray(est.get(b, g[b]), dtype=float) for b in g}
    world.perceived = dict(poses)  # the swing check's obstacles (clearance.swing_poses), updated by observe_block
    slot_of = {b: ls for ls, b in tower.slot_map.items()}
    return poses, {b: (slot_of[b], X) for b, X in poses.items()}, sorted(set(g) - set(est))


# ---------------------------------------------------------------- mid-turn (stage 3b): one block where it should be
PUSH_FRACTION = 0.4  # push.plan_push pushes the block 0.4 L (push_m = 0.4*L there): keep the two the same


def pushed_guess(X_WB: np.ndarray, cfg, push_station: int) -> np.ndarray:
    """Where a pushed block should be: its pose before the push, moved PUSH_FRACTION * L away from the push station
    (along that station's approach direction)."""
    X = np.array(X_WB, dtype=float)
    X[:2, 3] += PUSH_FRACTION * geo.block_dims(cfg)[0] * base.approach_dir(cfg, push_station)
    return X


def box_around(X_WB: np.ndarray, cfg, margin: float = 0.03):
    """(lower, upper) world box around a block at X_WB: a block length across in x and y (any turn), its height plus
    `margin` above and below.  The crop for a block away from the tower (on a pedestal)."""
    L, _, H = geo.block_dims(cfg)
    c, r = np.asarray(X_WB, float)[:3, 3], L / 2 + margin
    return c - [r, r, margin], c + [r, r, H + margin]


def observe_block(world, cfg, block_id: int, guess: np.ndarray, crop=None, label: str = "") -> np.ndarray:
    """One block's pose from the cameras, ICP started at `guess` (4x4 world).  If no camera finds it, the guess is
    used and a line is printed.  Updates world.perceived for the swing check."""
    est = sensing.estimate_block_poses(world.camera_images(), cfg, {block_id: guess}, crop=crop).get(block_id)
    if est is None:
        print(f"   [perception] {label}: block {block_id} not seen; using the expected pose")
        est = guess
    X = np.asarray(est, dtype=float)
    if getattr(world, "perceived", None) is not None:
        world.perceived[block_id] = X
    return X
