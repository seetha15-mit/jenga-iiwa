"""Extraction methods.  config.EXTRACTION_METHOD picks one.

A method module provides (see end_pinch.py, the default, for the full set):
    can_pull(tower, block_id, cfg)                      physical-reach rule: can this method take that block?
    choose_station(tower, block_id, cfg, current)       which base station to pull from
    can_place(tower, layer, slot, cfg)                  can this method put a block in that slot?
    choose_place_station(tower, layer, slot, cfg, current)
    plan_pull(X_RB, cfg)    -> [Waypoint]               THE TEAM WRITES THIS
    plan_place(X_RT, cfg)   -> [Waypoint]               THE TEAM WRITES THIS
Everything shared (tower state, legal pulls, collapse, logging) lives outside the methods.
"""

import importlib

REGISTRY = {"end_pinch": "jenga.methods.end_pinch", "push": "jenga.methods.push"}


def get_method(name: str):
    if name not in REGISTRY:
        raise ValueError(f"EXTRACTION_METHOD must be one of {sorted(REGISTRY)}, got {name!r}")
    return importlib.import_module(REGISTRY[name])
