"""RANDOM BASELINE (working).  Picks uniformly among the candidates."""

import numpy as np


def select_block(tower, candidates, cfg, rng, sensors=None) -> int:
    """Pick one candidate uniformly at random."""
    return int(rng.choice(np.array(candidates)))
