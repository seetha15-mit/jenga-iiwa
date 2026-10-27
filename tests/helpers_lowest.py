"""A test strategy: always the lowest block id.  Registered by tests/test_harness.py."""


def select_block(tower, candidates, cfg, rng, sensors=None):
    return int(min(candidates))
