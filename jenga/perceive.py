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
    world.held_grasp = None  # a new turn: nothing in the hand yet (record_grasp sets it)
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


# ---------------------------------------------------------------- the block in the hand (stage 3b)
def _mat(X) -> np.ndarray:
    return np.asarray(X.GetAsMatrix4() if hasattr(X, "GetAsMatrix4") else X, dtype=float)


def grasp_index(waypoints, cfg):
    """Index of the first waypoint whose opening is narrower than the one before: where the fingers close.
    None if they never close."""
    widths = [cfg.FINGER_OPEN_COMMAND_M if w.opening is None else w.opening for w in waypoints]
    return next((i for i in range(1, len(widths)) if widths[i] < widths[i - 1]), None)


def record_grasp(world, cfg, block_id: int, waypoints, X_RB) -> None:
    """Remember where the block sits in the hand when the fingers close: X_GB = inv(X_RG at the closing waypoint)
    X_RB, both in the robot frame and both from the plan and its target (no truth).  Works for any method's grasp.
    observe_held starts ICP from it; perception then finds how far the block has slipped since."""
    i = grasp_index(waypoints, cfg)
    if i is not None:
        world.held_grasp = (block_id, np.linalg.inv(_mat(waypoints[i].X_RG)) @ _mat(X_RB))


def observe_held(world, cfg, block_id: int, label: str = "") -> np.ndarray:
    """The block in the hand, from the cameras: ICP from (hand pose now) x (the grasp record_grasp kept), in a box
    around that guess.  Measured with the existing cameras (scratch/inhand_probe.py): seen at every moment checked,
    within 0.3 mm at the place."""
    grasp = getattr(world, "held_grasp", None)
    if grasp is None or grasp[0] != block_id:
        print(f"   [perception] {label}: no grasp recorded for block {block_id}; using the simulator's pose")
        return np.asarray(world.block_poses()[block_id], dtype=float)
    X_WG = np.asarray(world.X_WR(), float) @ _mat(world.kin.gripper_pose(world.arm_q()))
    guess = X_WG @ grasp[1]
    return observe_block(world, cfg, block_id, guess, crop=box_around(guess, cfg), label=label)
