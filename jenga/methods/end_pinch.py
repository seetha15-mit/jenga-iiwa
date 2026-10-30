"""END-PINCH (the default method): the hand comes in from the side, the jaw closes on the two END faces
of an EDGE block, the block slides out sideways, and is then placed on top as it is.

TWO KINDS OF CODE IN THIS FILE
  * Reach and station rules (top half): written already.  They are geometry only.
  * plan_pull and plan_place (bottom half): THE TEAM WRITES THESE.  Search for "CODE TO BE WRITTEN".

Why only edge blocks: a middle block cannot slide out sideways past its two neighbours.
Why only some stations: the hand holds the block by its ends, so the block's long axis is always
across the reach direction.  At a station the robot can therefore only work on layers whose long axis
is across its approach direction.

Why the far slot is skipped when placing (derived, untested in simulation): with a level hand the
50 mm tall hand body sits about 72 mm behind the pads.  Placing at the FAR slot of a layer puts the
body over the tower's near face and about 14 mm below the block's bottom plane, i.e. into the layer
below.  The near and middle slots are clear.  The far slot is filled from the opposite station, where
it is the near slot.  (geometry.housing_overlap has the numbers.)
"""

import numpy as np

from jenga import base
from jenga import geometry as geo
from jenga.waypoint import Waypoint  # noqa: F401  (the team's steps return lists of these)


# ====================================================================== rules (written)
def can_pull(tower, block_id, cfg) -> bool:
    """End-pinch can take a block only if it is an edge block and some station sees it as the near edge."""
    return len(base.near_edge_stations(tower, block_id)) > 0


def choose_station(tower, block_id, cfg, current_station: int) -> int:
    """The station to pull from.  Usually only one qualifies.  REPLACE THIS to move to option C
    (a freely moving base): return a base pose instead of an index."""
    options = base.near_edge_stations(tower, block_id)
    return base.nearest_station(cfg, options, current_station)


def place_stations(tower, layer: int, slot: int, cfg):
    """Stations from which a held block can be placed at (layer, slot) without the hand body
    hitting the tower (see the module docstring)."""
    across = tower.across_axis(layer)
    _, W, _ = geo.block_dims(cfg)
    a_off = (slot - 1) * (W + cfg.BLOCK_GAP_M)  # slot centre along the across axis
    out = []
    for k in range(len(cfg.STATION_ANGLES_DEG)):
        c = float(across @ base.approach_dir(cfg, k))
        if abs(abs(c) - 1.0) > 1e-6:  # approach must run along the across axis
            continue
        if not geo.housing_collides(cfg, a_off * c):
            out.append(k)
    return out


def can_place(tower, layer: int, slot: int, cfg) -> bool:
    return len(place_stations(tower, layer, slot, cfg)) > 0


def choose_place_station(tower, layer: int, slot: int, cfg, current_station: int) -> int:
    options = place_stations(tower, layer, slot, cfg)
    return base.nearest_station(cfg, options, current_station)


    


# ====================================================================== the team's steps
def plan_pull(X_RB, cfg):
    """Waypoints that take the hand from where it is, pinch the block by its ends, pull it out
    sideways, and finish holding it clear of the tower.

    Inputs
        X_RB : pydrake RigidTransform.  Pose of the block's body frame B in the ROBOT frame R.
               B's origin is the centre of the block's BOTTOM face; B's x axis is the long axis.
               R: x forward from the robot base, z up, origin on the arm's mounting flange.
        cfg  : the config module.  Useful values:
               geo.block_dims(cfg) -> (L, W, H) in m;  geo.jaw_margin_per_side_m(cfg);
               geo.finger_height_margin_per_side_m(cfg);  cfg.FINGER_OPEN_COMMAND_M;
               cfg.FINGER_SQUEEZE_M;  cfg.PAD_CENTRE_FROM_G_M;  cfg.WAYPOINT_SPEED_MS.
    Returns
        A list of Waypoint(X_RG, opening):  X_RG = pose of the hand frame G in R (RigidTransform);
        opening = finger command in m (None = fully open).  To close on the block, repeat a pose
        with a smaller opening.  The hand frame G: x = finger axis, y = reach axis (toward the
        fingertips), z = the remaining axis.  The pads are cfg.PAD_CENTRE_FROM_G_M along G y from G.

    Things to decide (these are the robotics choices you will be asked to justify):
        * the hand orientation: level, so G y is horizontal; G x is along the block's long axis
        * the approach path and how far back to start;  the pad height (block centre = z + H/2)
        * how hard to squeeze;  how far to pull out and how fast;  whether to lift a little
    Warnings (derived, untested in simulation)
        * The hand must be within about 0.9 mm (finger_height_margin_per_side_m) of the right
          height, or the pads touch the layers above and below the block.
        * Arm collisions are NOT checked (DECISIONS.md, open items).  Watch the replay.
        * The pset-style controller ignores joint limits; the executor warns if a joint ends near one.
    The next step, plan_place, starts from the last pose you return here.
    """
    # ---- BEGIN CODE TO BE WRITTEN ----
    from pydrake.all import RigidTransform, RotationMatrix  # here, not at the top: the quick tests import this file without Drake

    # Check orientation of the block being chosen
    L,W,H = geo.block_dims(cfg)
    bx = X_RB.rotation().matrix()[:, 0]
    if bx[1] < 0:
        X_RB = X_RB @ RigidTransform(RotationMatrix.MakeZRotation(np.pi), [0, 0, 0])
    R_BG = RotationMatrix.MakeXRotation(np.radians(-180))
    X_BG = RigidTransform(R_BG,[0,cfg.PAD_CENTRE_FROM_G_M,0.5*H])
    offset = RigidTransform([0,-0.1,0])
    upoffset = RigidTransform([0,0,-0.3])
    pickpoint = X_RB@X_BG
    pickpounce = pickpoint@offset
    wayplist = []
    wayplist.append(Waypoint(pickpounce, cfg.FINGER_OPEN_COMMAND_M))     # 1. get near, open
    wayplist.append(Waypoint(pickpoint,  cfg.FINGER_OPEN_COMMAND_M))
    wayplist.append(Waypoint(pickpoint,  L-cfg.FINGER_SQUEEZE_M))     # 2. move onto the block
    wayplist.append(Waypoint(pickpoint,  L-cfg.FINGER_SQUEEZE_M,1.0))# 3. SAME pose, close = grasp
    wayplist.append(Waypoint(pickpounce, L-cfg.FINGER_SQUEEZE_M,6.0))   # 4. retreat holding it
    wayplist.append(Waypoint(pickpounce@upoffset,L-cfg.FINGER_SQUEEZE_M))
    return wayplist
    
    # ---- END CODE TO BE WRITTEN ----


def plan_place(X_RT, cfg):
    """Waypoints that carry the held block to the top of the tower, set it down at its target
    slot, release it, and back the hand away.

    Inputs
        X_RT : pydrake RigidTransform.  The TARGET pose for the block's body frame in the ROBOT
               frame (same frame convention as X_RB above), at the slot on the top layer.
               The robot is already at a station from which this slot is reachable.
        cfg  : the config module.
    Returns
        A list of Waypoint(X_RG, opening), as for plan_pull.  The first waypoint starts from where
        plan_pull ended (block held, the hand is wherever you left it).

    Things to decide
        * how to get the block over the top layer without dragging the tower
        * how to lower it, how gently, and when to open the jaw;  how to retreat
        * the block is held by its ends, so the hand finishes level with the jaw across the block
    Warnings: as for plan_pull.  Also: the block is placed 'as it is', with no re-grasp.
    """
    # ---- BEGIN CODE TO BE WRITTEN ----
    from pydrake.all import RigidTransform, RotationMatrix  # here, not at the top: the quick tests import this file without Drake
    
    # Check orientation of the block being chosen
    L,W,H = geo.block_dims(cfg)
    bx = X_RT.rotation().matrix()[:, 0]
    if bx[1] < 0:
        X_RT = X_RT @ RigidTransform(RotationMatrix.MakeZRotation(np.pi), [0, 0, 0])
    R_TG = RotationMatrix.MakeXRotation(np.radians(-180))
    X_TG = RigidTransform(R_TG,[0,cfg.PAD_CENTRE_FROM_G_M,0.5*H])
    offset = RigidTransform([0,0,-0.2])
    placepoint = X_RT@X_TG
    placepounce = placepoint@offset
    wayplist = []
    wayplist.append(Waypoint(placepounce, L-cfg.FINGER_SQUEEZE_M))   
    wayplist.append(Waypoint(placepoint,  L-cfg.FINGER_SQUEEZE_M))    
    wayplist.append(Waypoint(placepoint,  cfg.FINGER_OPEN_COMMAND_M))
    wayplist.append(Waypoint(placepoint,  cfg.FINGER_OPEN_COMMAND_M, 1.0))   
    wayplist.append(Waypoint(placepounce, cfg.FINGER_OPEN_COMMAND_M))   
    return wayplist
    # ---- END CODE TO BE WRITTEN ----
