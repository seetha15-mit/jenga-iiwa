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
    slot_of = {b: ls for ls, b in tower.slot_map.items()}
    return poses, {b: (slot_of[b], X) for b, X in poses.items()}, sorted(set(g) - set(est))
