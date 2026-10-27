"""A place for the hand to be.  Plain Python: the team's steps build lists of these."""

from typing import NamedTuple, Optional


class Waypoint(NamedTuple):
    """X_RG: pose of the hand frame G in the ROBOT frame R (a pydrake RigidTransform or a 4x4 numpy array).
    opening: commanded finger opening in meters at this waypoint; None means config.FINGER_OPEN_COMMAND_M.
    (The pad gap is the opening + 5 mm.)  To close on a block, repeat the pose with a smaller opening."""

    X_RG: object
    opening: Optional[float] = None
