"""FORCE-GUIDED SELECTION  (TEAM STUB)

Probes blocks and reads the simulated wrist force (course approach (d)) to find loose ones.
Runs on varied-friction towers (config.FRICTION_MODE = "random").  Facts to respect (see jenga/sensing.py):
the reading is in the GRIPPER frame and includes the hand's own weight (10.7 N at rest, measured).
Per-block friction and exact contact forces are hidden: never read them.
"""


def select_block(tower, candidates, cfg, rng, sensors=None) -> int:
    """Choose a block using what the wrist force sensor reports while probing.

    Inputs and output: see jenga/strategies/__init__.py.
    How probes are commanded and how forces come back is for the team to decide (jenga/sensing.py).
    """
    # ---- BEGIN CODE TO BE WRITTEN ----
    raise NotImplementedError("TODO: write select_block for force_guided (see docs/FOR_THE_TEAM.md)")
    # ---- END CODE TO BE WRITTEN ----
