"""What happened to the block we tried to move?  EVALUATION SIDE (uses true poses).

Outcomes of a turn: "replaced", "failed_pull", "dropped".  ("collapse" is decided separately by the
collapse detector and overrides these.)
"""

import numpy as np


def classify_turn(pose_before, pose_after, target_pose, cfg) -> str:
    """replaced   = the block ended within PLACE_TOLERANCE_M of its target slot on top.
    failed_pull = it is still within PULL_MOVED_M of where it started (it never came out).
    dropped     = anything else: it moved but did not arrive; it stays wherever it fell."""
    to_target = float(np.linalg.norm(pose_after[:3, 3] - target_pose[:3, 3]))
    if to_target <= cfg.PLACE_TOLERANCE_M:
        return "replaced"
    moved = float(np.linalg.norm(pose_after[:3, 3] - pose_before[:3, 3]))
    if moved < cfg.PULL_MOVED_M:
        return "failed_pull"
    return "dropped"
