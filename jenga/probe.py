"""Probe before taking a block (D-27): nudge it PROBE_DIST_M, read the wrist force, and back off if it resists.

A probed turn, run by mover.RealMover when the harness asks for a probe.  After the plan's first (joint) move,
reach[1:] + probe + go_on run as ONE straight-line motion, so the hand does not stop at the block (stopping there and
restarting dragged the layer below, D-27):
    reach     the method's own waypoints up to contact (push: fingertips on the end face; end-pinch: block gripped)
    tare      the wrist-force sample as the hand reaches the end of reach: the hand's weight and any contact so far
    probe     a slow straight move of PROBE_DIST_M (push: into the tower; end-pinch: out), wrist force sampled
    then      at the probe's end: go on with go_on (the motion just continues) if the block gave way, else the motion
              stops there and back_off runs (let go, retreat)  (executor.run_waypoints stop_at)
A rejected block keeps its slot (it is left up to PROBE_DIST_M out of place); the harness drops it from this turn's
candidates and asks the strategy again (harness.run_trial).

Resistance = the largest change in the translational wrist force from the tare during the probe.  The hand does not
turn during a probe, so its weight stays in the tare and cancels (sensing.py: the raw reading includes it).
Plain Python: the plans come from the methods (push.plan_probe, end_pinch.plan_probe).
"""

from typing import List, NamedTuple, Optional

import numpy as np


class ProbePlan(NamedTuple):
    """Four waypoint lists for one probed push or pull (see the module docstring)."""

    reach: List  # run with executor.run_plan (its first waypoint is a joint move)
    probe: List  # straight-line, force sampled
    go_on: List  # straight-line, from the probed pose: the rest of the method's push or pull
    back_off: List  # straight-line, from the probed pose: let go and retreat


class ProbeRejected(Exception):
    """The probed block resisted more than its threshold (probe.max_force); the hand has let go and backed off."""

    def __init__(self, block_id: int, resistance_n: float):
        super().__init__(f"probe: block {block_id} resisted {resistance_n:.2f} N")
        self.block_id = block_id
        self.resistance_n = resistance_n


def resistance(trace, tare) -> float:
    """Largest change of the translational wrist force from the tare, in N.

    Inputs: trace (list of [Fx Fy Fz Tx Ty Tz] readings sampled during the probe), tare (one such reading taken at
    rest just before it).  Returns 0.0 for an empty trace."""
    if len(trace) == 0:
        return 0.0
    F = np.asarray(trace, float)[:, :3] - np.asarray(tare, float)[:3]
    return float(np.max(np.linalg.norm(F, axis=1)))


def max_force(cfg, step: str):
    """The rejection threshold for this probe step, in N, or None (never reject): PROBE_MAX_PULL_N for an end-pinch
    pull, PROBE_MAX_PUSH_N for a push.  Two settings because the contacts differ (a gripped pull, a fingertip push),
    so one number could end up choosing by method, not by load."""
    return cfg.PROBE_MAX_PULL_N if step == "pull" else cfg.PROBE_MAX_PUSH_N


def accept(resistance_n: float, cfg, step: str) -> bool:
    """True = take the block.  No threshold for this step = measure only, never reject."""
    m = max_force(cfg, step)
    return m is None or resistance_n <= m


def fallback(rejected: dict) -> Optional[int]:
    """The block to take, without a probe, when the turn has run out of probes or candidates: the one that resisted
    least (rejected: block id -> resistance in N).  None if nothing was rejected."""
    if not rejected:
        return None
    return min(rejected, key=lambda b: (rejected[b], b))
