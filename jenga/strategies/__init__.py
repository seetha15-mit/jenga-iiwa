"""Block-selection strategies.  config.STRATEGY picks one.

Every strategy is one function with the same signature:

    select_block(tower, candidates, cfg, rng, sensors=None) -> block_id

  tower       jenga.tower.TowerState: the exact tower (block poses, layers, slots).  Initial strategies may
              use all of it (the rule-based one does).
  candidates  block ids that are legal to pull AND that the extraction method can physically take.
              Return one of these.
  cfg         the config module.
  rng         numpy Generator, seeded per trial (use it for any randomness, never numpy's global one).
  sensors     reserved for the perception and force strategies (the team decides what goes here).

HIDDEN from every strategy: per-block friction and exact contact forces.  Never import from
jenga.evaluation or read the simulator's truth.
The harness picks the FIRST block of every trial at random itself (course rule), whatever the strategy.
"""

import importlib

REGISTRY = {
    "random": "jenga.strategies.random_baseline",
    "rule_based": "jenga.strategies.rule_based",
    "perception": "jenga.strategies.perception",
    "force_guided": "jenga.strategies.force_guided",
}


def get_strategy(name: str):
    if name not in REGISTRY:
        raise ValueError(f"STRATEGY must be one of {sorted(REGISTRY)}, got {name!r}")
    return importlib.import_module(REGISTRY[name]).select_block
