"""PUSH method (possibly with a push tool).  Only the station rules are written.

The robot faces a block's SHORT end face and pushes along the block's long axis, so the block slides out
the far side.  Any slot works (the middle too) and either end works (base.push_stations).

A turn has four steps, in this order (jenga/mover.py RealMover runs them):
    plan_push     at the push station (choose_station): push the block part way out the far side
    plan_pull     at the OPPOSITE station (choose_pull_station): grasp the end that sticks out, pull it clear
    plan_regrab   at the same station: change the grasp so the block can be placed on top
    plan_place    at the place station (choose_place_station): carry, lower, release
The base rolls between the stations with the arm in its carry pose, so the hand is empty while it rolls
from push to pull.

WRITTEN:      can_pull, choose_station, choose_pull_station (which blocks can be pushed, and from where).
PLACEHOLDERS: can_place, choose_place_station, plan_push, plan_pull, plan_regrab, plan_place.  The tool,
              the tolerances, how far to push, and how the block is regrabbed and placed are undecided
              (DECISIONS.md O-5).  Plans return lists of Waypoint, as in jenga/methods/end_pinch.py.
"""
import numpy as np
from jenga import base
from jenga import geometry as geo
from jenga.waypoint import Waypoint

_MSG = "This part of the push method is not implemented yet (decided later; see DECISIONS.md O-5)."


# ====================================================================== rules (written)
def can_pull(tower, block_id, cfg) -> bool:
    """Push can take a block if some station faces its end face."""
    return len(base.push_stations(tower, block_id)) > 0


def choose_station(tower, block_id, cfg, current_station) -> int:
    """The station to push from: of the two that face the block's ends, the nearer one."""
    return base.nearest_station(cfg, base.push_stations(tower, block_id), current_station)


def choose_pull_station(tower, block_id, cfg, push_station) -> int:
    """The station to pull from: the other end of the block, where the push made it stick out."""
    return next(k for k in base.push_stations(tower, block_id) if k != push_station)

def can_place(tower, layer, slot, cfg) -> bool:
    return True


def choose_place_station(tower, layer, slot, cfg, current_station) -> int:
    return current_station


# ====================================================================== the team's steps (placeholders)
def plan_push(X_RB, cfg):
    """Waypoints that push the block along its long axis from the near end, so it slides out the far side.
    X_RB: the block's pose in the robot frame at the push station (same conventions as end_pinch.plan_pull)."""
    from pydrake.all import RigidTransform, RotationMatrix
    L,W,H = geo.block_dims(cfg)
    bx = X_RB.rotation().matrix()[:, 0]
    if bx[0] < 0:
      X_RB = X_RB @ RigidTransform(RotationMatrix.MakeZRotation(np.pi), [0, 0, 0])
    R_BG = RotationMatrix.MakeXRotation(np.radians(-180))@RotationMatrix.MakeZRotation(np.radians(-90))
    X_BG = RigidTransform(R_BG,[-L/2-cfg.FINGERTIP_REACH_M,cfg.PAD_CENTRE_FROM_G_M/2,0.5*H])
    offset = RigidTransform([0,-0.1,0])
    upoffset = RigidTransform([0,0,-0.3])
    push = RigidTransform([0,L/4,0])
    pickpoint = X_RB@X_BG
    pickpounce = pickpoint@offset
    pickpush = pickpoint@push
    wayplist = []
    wayplist.append(Waypoint(pickpounce, cfg.FINGER_OPEN_COMMAND_M))     # 1. get near, open
    wayplist.append(Waypoint(pickpoint,  cfg.FINGER_OPEN_COMMAND_M))
    wayplist.append(Waypoint(pickpush,  cfg.FINGER_OPEN_COMMAND_M,2))     # 2. move onto the block
    wayplist.append(Waypoint(pickpounce, cfg.FINGER_OPEN_COMMAND_M))   # 4. retreat holding it
    wayplist.append(Waypoint(pickpounce@upoffset,cfg.FINGER_OPEN_COMMAND_M))
    return wayplist


def plan_pull(X_RB, cfg):
    """Waypoints that grasp the end sticking out of the far side and pull the block clear of the tower.
    X_RB: the block's pose AFTER the push, in the robot frame at the pull station."""
    from pydrake.all import RigidTransform, RotationMatrix
    L,W,H = geo.block_dims(cfg)
    bx = X_RB.rotation().matrix()[:, 0]
    if bx[0] < 0:
      X_RB = X_RB @ RigidTransform(RotationMatrix.MakeZRotation(np.pi), [0, 0, 0])
    R_BG = RotationMatrix.MakeXRotation(np.radians(-180))@RotationMatrix.MakeZRotation(np.radians(-90))
    X_BG = RigidTransform(R_BG,[-L/2-cfg.FINGERTIP_REACH_M+L/5,0,0.5*H])
    offset = RigidTransform([0,-0.2,0])
    pickback = RigidTransform([0,-0.05,0])
    upgroundoffset = RigidTransform([0,0,X_RB.translation()[2]-0.115])
    groundoffset = RigidTransform([0,0,X_RB.translation()[2]-0.015])
    pickpoint = X_RB@X_BG
    pickpounce = pickpoint@offset
    wayplist = []
    wayplist.append(Waypoint(pickpounce, cfg.FINGER_OPEN_COMMAND_M))     # 1. get near, open
    wayplist.append(Waypoint(pickpoint,  cfg.FINGER_OPEN_COMMAND_M))
    wayplist.append(Waypoint(pickpoint,  W-1.5*cfg.FINGER_SQUEEZE_M))
    wayplist.append(Waypoint(pickpoint,  W-1.5*cfg.FINGER_SQUEEZE_M,1))      # 2. move onto the block
    wayplist.append(Waypoint(pickpoint@pickback, W-1.5*cfg.FINGER_SQUEEZE_M,0.05))   # 4. retreat holding it
    wayplist.append(Waypoint(pickpounce, W-1.5*cfg.FINGER_SQUEEZE_M))
    wayplist.append(Waypoint(pickpounce@groundoffset,  W-1.5*cfg.FINGER_SQUEEZE_M))
    wayplist.append(Waypoint(pickpounce@groundoffset,  cfg.FINGER_OPEN_COMMAND_M))
    wayplist.append(Waypoint(pickpounce@upgroundoffset,  cfg.FINGER_OPEN_COMMAND_M))
    return wayplist


def plan_regrab(X_RB, cfg):
    """Waypoints that change the grasp so the block can be placed on top (e.g. set it down and pick it up again).
    X_RB: the block's pose after the pull, in the robot frame at the same station."""
    from pydrake.all import RigidTransform, RotationMatrix
    L,W,H = geo.block_dims(cfg)
    R_BG = RotationMatrix.MakeXRotation(np.radians(-90))
    X_BG = RigidTransform(R_BG,[0,0,0.5*H+cfg.PAD_CENTRE_FROM_G_M])
    offset = RigidTransform([0,-0.2,0])
    upoffset = RigidTransform([0,-0.1,0])
    pickpoint = X_RB@X_BG
    pickpounce = pickpoint@offset
    wayplist = []
    wayplist.append(Waypoint(pickpounce, cfg.FINGER_OPEN_COMMAND_M))     # 1. get near, open
    wayplist.append(Waypoint(pickpoint,  cfg.FINGER_OPEN_COMMAND_M))
    wayplist.append(Waypoint(pickpoint,  L-2*cfg.FINGER_SQUEEZE_M))
    wayplist.append(Waypoint(pickpoint,  L-2*cfg.FINGER_SQUEEZE_M,1.0))      # 2. move onto the block
    wayplist.append(Waypoint(pickpounce, L-2*cfg.FINGER_SQUEEZE_M))   # 4. retreat holding it
    wayplist.append(Waypoint(pickpounce@upoffset,L-2*cfg.FINGER_SQUEEZE_M))
    return wayplist


def plan_place(X_RT, cfg):
    """Waypoints that carry the block over the tower, set it down at its target slot and release it.
    X_RT: the target pose on top, in the robot frame at the place station."""
    from pydrake.all import RigidTransform, RotationMatrix  # here, not at the top: the quick tests import this file without Drake
        
    # Check orientation of the block being chosen
    L,W,H = geo.block_dims(cfg)
    R_TG = RotationMatrix.MakeXRotation(np.radians(-90))
    X_TG = RigidTransform(R_TG,[0,0,0.5*H+cfg.PAD_CENTRE_FROM_G_M])
    offset = RigidTransform([0,-0.05,0])
    placepoint = X_RT@X_TG
    placepounce = placepoint@offset
    wayplist = []
    wayplist.append(Waypoint(placepounce, L-2*cfg.FINGER_SQUEEZE_M))   
    wayplist.append(Waypoint(placepoint,  L-2*cfg.FINGER_SQUEEZE_M))    
    wayplist.append(Waypoint(placepoint,  cfg.FINGER_OPEN_COMMAND_M))
    wayplist.append(Waypoint(placepoint,  cfg.FINGER_OPEN_COMMAND_M, 1.0))   
    wayplist.append(Waypoint(placepounce, cfg.FINGER_OPEN_COMMAND_M))   
    return wayplist
