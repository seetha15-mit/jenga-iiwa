"""RULE-BASED SELECTION  (TEAM STUB)

Chooses a block using the exact tower state (course approach (b)): for example favour outside blocks
in upper layers, or blocks whose neighbours give the most support.  Runs on equal-friction towers
(and also on varied-friction towers, as a comparison for the force strategy).
"""


def select_block(tower, candidates, cfg, rng, sensors=None) -> int:
    """Choose a block using the exact tower state.

    Inputs and output: see jenga/strategies/__init__.py.
    You may use everything in `tower`: tower.blocks (poses, layer, slot), tower.layer_blocks(layer),
    tower.top_layer().  Not allowed: per-block friction, contact forces.
    """
    # ---- BEGIN CODE TO BE WRITTEN ----
    raise NotImplementedError("TODO: write select_block for rule_based (see docs/FOR_THE_TEAM.md)")
    # ---- END CODE TO BE WRITTEN ----
