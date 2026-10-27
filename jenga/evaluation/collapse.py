"""Collapse detection.  EVALUATION SIDE: it reads ground truth and returns only continue-or-stop.

The robot and the strategies never see this verdict or its inputs.  The harness uses it to end a trial.

To write the advanced detector (the team's job): implement `advanced` below and set
COLLAPSE_DETECTOR = "advanced" in config.py.  It receives a Truth with everything the simulator knows
(true block poses before and after the turn, the tower bookkeeping, and later contact forces).  The basic
detector deliberately ignores most of it.
"""

from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np


@dataclass
class Truth:
    poses_before: Dict[int, np.ndarray]  # block_id -> 4x4 world pose before the turn (true)
    poses_after: Dict[int, np.ndarray]  # same, after the turn (true)
    tower: object  # jenga.tower.TowerState (bookkeeping: layer, slot, loose)
    moved_block_id: int  # the block this turn tried to move; it is allowed to have moved
    contact_forces: Optional[object] = None  # not wired yet; reserved for the advanced detector


@dataclass
class Verdict:
    stop: bool  # True = the tower has collapsed, end the trial
    reason: str = ""


def basic(truth: Truth, cfg) -> Verdict:
    """The course rule: any block (other than the one we moved) that has moved more than
    COLLAPSE_DISPLACEMENT_M since before the turn means the tower has collapsed.
    Uses poses only.  Dropped ('loose') blocks are ignored."""
    worst_id, worst = None, 0.0
    for bid, before in truth.poses_before.items():
        if bid == truth.moved_block_id or truth.tower.blocks[bid].loose:
            continue
        d = float(np.linalg.norm(truth.poses_after[bid][:3, 3] - before[:3, 3]))
        if d > worst:
            worst_id, worst = bid, d
    if worst > cfg.COLLAPSE_DISPLACEMENT_M:
        return Verdict(True, f"block {worst_id} moved {worst * 1000:.1f} mm")
    return Verdict(False, "")


def advanced(truth: Truth, cfg) -> Verdict:
    """The team's collapse detector.  Same inputs and outputs as `basic`.

    Ideas to consider: tilt as well as position, blocks falling below the platform, how many blocks
    moved at all, a time window instead of one before/after comparison, contact forces.
    """
    # ---- BEGIN CODE TO BE WRITTEN ----
    raise NotImplementedError("TODO: implement the advanced collapse detector (see docs/FOR_THE_TEAM.md)")
    # ---- END CODE TO BE WRITTEN ----


DETECTORS = {"basic": basic, "advanced": advanced}


def get_detector(name: str):
    if name not in DETECTORS:
        raise ValueError(f"COLLAPSE_DETECTOR must be one of {sorted(DETECTORS)}, got {name!r}")
    return DETECTORS[name]
