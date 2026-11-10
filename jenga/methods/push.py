"""PUSH method: push a block part way out with the fingertips, pull it from the other side, set it on a
pedestal, pick it up by its ends and place it on top (DECISIONS.md D-21, D-22, O-5).

The robot faces a block's SHORT end face and pushes along the block's long axis, so the block slides out
the far side.  Any slot works (the middle too) and either end works (base.push_stations).

A turn has five steps, in this order (jenga/mover.py RealMover runs them, each starting with a joint move):
    plan_push       at the push station (choose_station): push the block 0.4 L out the far side
    plan_pull       at the OPPOSITE station (choose_pull_station): grip the end that sticks out, pull it clear
    plan_ped_place  at the same station: set it on the pedestal on the robot's left (D-22)
    plan_regrab     at the same station: pick it up level by its ends (end_pinch.plan_pull)
    plan_place      at the place station (choose_place_station): carry, lower, release (end_pinch.plan_place)
The base rolls between the stations with the arm in its carry pose.

Rules (which blocks, which stations) were written with Claude; the plans are Katie's.  Measured status and
limits (tuned on block 1; needs two or more layers above the pulled block) are in DECISIONS.md O-5.
Plans return lists of Waypoint, as in jenga/methods/end_pinch.py.
"""
import numpy as np
from jenga import base
from jenga import geometry as geo
from jenga.waypoint import Waypoint
from jenga.methods import end_pinch



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
    return end_pinch.can_place(tower, layer, slot, cfg)          # the block is held end-pinch after the regrab


def choose_place_station(tower, layer, slot, cfg, current_station) -> int:
    return end_pinch.choose_place_station(tower, layer, slot, cfg, current_station)


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
    push = RigidTransform([0,0.4*L,0])
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
    lift = RigidTransform([0, 0, -0.00015])
    upoffset = RigidTransform([0,0,-0.03])
    
    pickpoint = X_RB@X_BG
    pickpounce = pickpoint@offset
    wayplist = []
    wayplist.append(Waypoint(pickpounce, cfg.FINGER_OPEN_COMMAND_M))     # 1. get near, open
    wayplist.append(Waypoint(pickpoint,  cfg.FINGER_OPEN_COMMAND_M))
    wayplist.append(Waypoint(pickpoint,  W-1.75*cfg.FINGER_SQUEEZE_M))
    wayplist.append(Waypoint(pickpoint@lift,  W-1.75*cfg.FINGER_SQUEEZE_M,1))      # 2. move onto the block
    wayplist.append(Waypoint(pickpounce@lift, W-1.75*cfg.FINGER_SQUEEZE_M))
    wayplist.append(Waypoint(pickpounce@upoffset, W-1.75*cfg.FINGER_SQUEEZE_M))
    return wayplist

def plan_ped_place(X_RB, cfg):
    """Waypoints that carry the pulled block to the pedestal on the robot's left (D-22), set it down with its
    long axis pointing at the tower, let go and rise.  The first waypoint is a joint move, which also puts the
    arm in a configuration that can reach the pedestal; plan_regrab then picks the block up level by its ends.
    X_RB: the block's pose in the hand after plan_pull (unused: the pedestal pose comes from config)."""
    from pydrake.all import RigidTransform, RotationMatrix
    L,W,H = geo.block_dims(cfg)
    d = cfg.PEDESTAL_DISTANCE_M / np.sqrt(2)
    R_RBped = RotationMatrix.MakeZRotation(-np.pi/4)
    # Release 2 mm BELOW the post top with a +1 mm target
    X_RBped = RigidTransform(R_RBped,[cfg.STATION_STANDOFF_M - d, d, cfg.PEDESTAL_HEIGHT_M - 0.002])
    R_BG = RotationMatrix.MakeXRotation(np.radians(-180))@RotationMatrix.MakeZRotation(np.radians(-90))
    X_BG = RigidTransform(R_BG,[-L/2-cfg.FINGERTIP_REACH_M+L/5,0,0.5*H])
    setpoint = X_RBped @ X_BG
    above = RigidTransform([0, 0, -0.05])
    setpounce = setpoint@above
    grip = W-1.75*cfg.FINGER_SQUEEZE_M
    touch = W - 0.005                                # pad gap = opening + 5 mm, so this is pads just touching
    # Release the squeeze fully on a still leg then open in steps, and lift only once open
    wayplist = []
    wayplist.append(Waypoint(setpounce, grip))
    wayplist.append(Waypoint(setpoint,  grip))
    wayplist.append(Waypoint(setpoint,  touch + 0.002))                      # relax: pads ~1 mm clear each side
    wayplist.append(Waypoint(setpoint,  touch + 0.010, 0.75))                # still: squeeze released, block on post
    wayplist.append(Waypoint(setpoint,  cfg.FINGER_OPEN_COMMAND_M, 0.5))     # still: small opening
    wayplist.append(Waypoint(setpoint,  cfg.FINGER_OPEN_COMMAND_M, 0.5))     # still: full open
    wayplist.append(Waypoint(setpounce, cfg.FINGER_OPEN_COMMAND_M))          # only now lift
    return wayplist


def plan_regrab(X_RB, cfg):
    """Pick the block up off the pedestal by its ends (D-22): the same grasp end-pinch uses in the tower."""
    
    return end_pinch.plan_pull(X_RB, cfg)


def plan_place(X_RT, cfg):
    return end_pinch.plan_place(X_RT, cfg)
