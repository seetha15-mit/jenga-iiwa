"""A place for the hand to be.  Plain Python: the team's steps build lists of these."""

from typing import NamedTuple, Optional


class Waypoint(NamedTuple):
    """X_RG: pose of the hand frame G in the ROBOT frame R (a pydrake RigidTransform or a 4x4 numpy array).
    opening: commanded finger opening in meters at this waypoint; None means config.FINGER_OPEN_COMMAND_M.
    (The pad gap is the opening + 5 mm.)  To close on a block, repeat the pose with a smaller opening.
    seconds: how long the leg that ARRIVES at this waypoint takes; None means distance / speed
    (controller.waypoint_times).  Use it to slow one leg (a pull) or to give the fingers time to
    close at a repeated pose; the opening of a waypoint is commanded from the start of the NEXT leg.
    """

    X_RG: object
    opening: Optional[float] = None
    seconds: Optional[float] = None
