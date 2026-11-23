"""MIXED method: push for every middle block, end-pinch for every edge block (DECISIONS D-07).

Why: end-pinch cannot take a middle block, and pushing an edge block moved a 6-layer tower 11.4 mm against
0.27 mm for end-pinch on the same tower (measured, one trial each).  So each block goes to the method that
handles it best.

This module only answers the rules the harness asks (can_pull, can_place); RealMover asks method_for which
method runs the turn and then runs that method's own steps.  Placing is end-pinch's either way: push holds
the block end-pinch after its regrab (D-22).
"""
from jenga.methods import end_pinch, push


def method_for(tower, block_id):
    """The method that takes this block: push for a middle block (slot 1), end-pinch for an edge block."""
    return push if tower.blocks[block_id].slot == 1 else end_pinch


def can_pull(tower, block_id, cfg) -> bool:
    return method_for(tower, block_id).can_pull(tower, block_id, cfg)


def can_place(tower, layer, slot, cfg) -> bool:
    return end_pinch.can_place(tower, layer, slot, cfg)


def choose_place_station(tower, layer, slot, cfg, current_station) -> int:
    return end_pinch.choose_place_station(tower, layer, slot, cfg, current_station)
