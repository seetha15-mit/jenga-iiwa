"""PUSH method (possibly with a push tool).  NOT IMPLEMENTED: a placeholder so the architecture has a slot.

Details (tool or no tool, tolerances, how the block is finally grasped) are decided later.  When that
happens, give this module the same functions as jenga/methods/end_pinch.py.  Nothing else changes:
tower state, legal pulls, collapse detection and logging are shared.
"""

_MSG = "The push method is not implemented yet (decided later; see DECISIONS.md)."


def can_pull(tower, block_id, cfg) -> bool:
    raise NotImplementedError(_MSG)


def choose_station(tower, block_id, cfg, current_station) -> int:
    raise NotImplementedError(_MSG)


def can_place(tower, layer, slot, cfg) -> bool:
    raise NotImplementedError(_MSG)


def choose_place_station(tower, layer, slot, cfg, current_station) -> int:
    raise NotImplementedError(_MSG)


def plan_pull(X_RB, cfg):
    raise NotImplementedError(_MSG)


def plan_place(X_RT, cfg):
    raise NotImplementedError(_MSG)
