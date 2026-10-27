"""SENSING: notes for the team.  NO DESIGN CHOICES HAVE BEEN MADE here; the person who owns the force
and perception strategies decides the interface, the weight compensation and the camera setup.

Facts (see DECISIONS.md for sources)
  * Wrist force: the simulator can report the reaction at the weld between the arm and the gripper
    (Sim.wrist_force_raw(), below).  The reading is in the GRIPPER frame G (measured: the hand's weight
    shows up on G y), not in the world frame.
  * The arm links have gravity switched off (an ideal gravity-compensated arm); the gripper keeps its weight.
  * It includes the hand's own weight: |F| = 10.7 N at rest with the hand pointing down (measured).  As the
    hand rotates, that weight moves between axes, so a strategy that wants the force from a block must
    deal with it.  Nothing here subtracts it.
  * Per-block friction and exact contact forces are HIDDEN from every strategy (requirement R-05).  Never read them.
  * The reading is polled between short AdvanceTo calls (jenga.scene.Sim.advance_logged), not wired as a
    port, because the hardware station does not export the plant's reaction-force port.
  * Set config.ENABLE_WRIST_FORCE_LOG = True to write a raw force trace per trial next to the replay.
  * Cameras: NOT set up.  The scenario `cameras:` block was written once in an earlier prototype and never run.
    The TA's advice: use several cameras to reduce partial observability; perception is harder than expected.
  * Probing with the force strategy needs motions: how a strategy commands them is open (the executor
    plays fixed waypoint lists, so it cannot stop the instant a force threshold is crossed; if the force
    strategy needs that, a stop condition would be added to jenga/executor.py without changing the steps).
"""

import numpy as np


def read_wrist_force_raw(world) -> np.ndarray:
    """[Fx, Fy, Fz, Tx, Ty, Tz] in the gripper frame, raw (includes the hand's weight, no noise)."""
    return world.wrist_force_raw()
