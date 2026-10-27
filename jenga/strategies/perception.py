"""PERCEPTION-BASED SELECTION  (TEAM STUB)

Estimates the tower state from depth-camera point clouds (course approach (c)), then chooses.
NOT SET UP: cameras are off (config.ENABLE_CAMERAS) and the scenario `cameras:` block has never been
run on our machines.  The TA warned perception is harder than expected: decide early whether to finish
this or the force strategy.
"""


def select_block(tower, candidates, cfg, rng, sensors=None) -> int:
    """Choose a block from a state estimated with cameras.

    Inputs and output: see jenga/strategies/__init__.py.
    `sensors` is where camera images would arrive; the team decides the interface.  Using `tower` here
    would defeat the point of the strategy.
    """
    # ---- BEGIN CODE TO BE WRITTEN ----
    raise NotImplementedError("TODO: write select_block for perception (see docs/FOR_THE_TEAM.md)")
    # ---- END CODE TO BE WRITTEN ----
