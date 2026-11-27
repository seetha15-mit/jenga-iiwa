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
The base rolls between the stations with the arm in its carry pose, except on the way to the place (D-02).

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
    """Push can take a block if some station faces its end face; with config.PUSH_MIDDLE_ONLY, only a middle
    block (slot 1), since pushing an edge block disturbs the tower far more (D-07); and only with at least
    config.PUSH_MIN_LAYERS_ABOVE complete layers above it (O-5); and with config.PUSH_MAX_LAYER set, only up to
    that layer (push is tuned on layers 0-3)."""
    if cfg.PUSH_MIDDLE_ONLY and tower.blocks[block_id].slot != 1:
        return False
    layer = tower.blocks[block_id].layer
    if cfg.PUSH_MAX_LAYER is not None and layer is not None and layer > cfg.PUSH_MAX_LAYER:
        return False
    if layers_above(tower, block_id) < cfg.PUSH_MIN_LAYERS_ABOVE:
        return False
    return len(base.push_stations(tower, block_id)) > 0


def layers_above(tower, block_id) -> int:
    """How many complete layers lie above this block's layer (0 for a loose block).  Their weight holds the layer
    right above in place while the block slides out (config.PUSH_MIN_LAYERS_ABOVE)."""
    layer = tower.blocks[block_id].layer
    if layer is None:
        return 0
    return sum(tower.is_complete(k) for k in range(layer + 1, tower.top_layer() + 1))


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
    push_m, push_s = 0.4*L, 2.0
    push = RigidTransform([0,push_m,0])
    clear_m = 0.010                                  # [estimate] fingertips this far outside the tower face before speeding up
    clear = RigidTransform([0,-clear_m,0])
    pickpoint = X_RB@X_BG
    pickpounce = pickpoint@offset
    pickpush = pickpoint@push
    wayplist = []
    wayplist.append(Waypoint(pickpounce, cfg.FINGER_OPEN_COMMAND_M))     # 1. get near, open
    wayplist.append(Waypoint(pickpoint,  cfg.FINGER_OPEN_COMMAND_M))
    wayplist.append(Waypoint(pickpush,  cfg.FINGER_OPEN_COMMAND_M,push_s))     # 2. push the block
    # 3. back out of the slot at the push's own speed: at full speed the hand ended 1.3 mm low and dragged the
    # layer below 26 mm (block 10, push sweep rerun, measured); speed up only once the fingertips are outside
    wayplist.append(Waypoint(pickpoint@clear, cfg.FINGER_OPEN_COMMAND_M, (push_m + clear_m) * push_s / push_m))
    wayplist.append(Waypoint(pickpounce, cfg.FINGER_OPEN_COMMAND_M))   # 4. retreat
    wayplist.append(Waypoint(pickpounce@upoffset,cfg.FINGER_OPEN_COMMAND_M))
    return wayplist


PUSH_AT_BLOCK = 2  # plan_push's first PUSH_AT_BLOCK waypoints end with the fingertips on the end face; keep in step with plan_push


def plan_probe(X_RB, cfg):
    """plan_push split for the probe (D-28, jenga/probe.py): fingertips to the end face, push PROBE_DIST_M in slowly,
    then either finish plan_push or back out.  Written with Claude; the push itself is plan_push's, unchanged.

    Inputs: X_RB, cfg as plan_push.  Returns: probe.ProbePlan(reach, probe, go_on, back_off).
    reach = plan_push up to the end face; probe = the same pose moved PROBE_DIST_M along G y (plan_push's own push
    direction) in PROBE_SECONDS; go_on = the rest of plan_push (the push leg keeps its time, so it is slightly
    slower); back_off = plan_push's own back-out after the push (slow until the fingertips are outside, then
    retreat and rise)."""
    from pydrake.all import RigidTransform
    from jenga.probe import ProbePlan
    wps = plan_push(X_RB, cfg)
    reach, go_on = wps[:PUSH_AT_BLOCK], wps[PUSH_AT_BLOCK:]
    at = reach[-1]
    probe = [Waypoint(at.X_RG @ RigidTransform([0, cfg.PROBE_DIST_M, 0]), at.opening, cfg.PROBE_SECONDS)]
    return ProbePlan(reach, probe, go_on, wps[PUSH_AT_BLOCK + 1:])


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
    # the pull-out in two parts on one straight line: while the block is still in the tower (0.6 L after a 0.4 L
    # push, plus 10 mm) at PULL_INSIDE_SPEED_MS, then the rest at PULL_OUT_SPEED_MS
    inside_m = 0.6*L + 0.010
    clear = pickpoint@RigidTransform([0,-inside_m,0])
    wayplist = []
    wayplist.append(Waypoint(pickpounce, cfg.FINGER_OPEN_COMMAND_M))     # 1. get near, open
    wayplist.append(Waypoint(pickpoint,  cfg.FINGER_OPEN_COMMAND_M))
    wayplist.append(Waypoint(pickpoint,  W-1.75*cfg.FINGER_SQUEEZE_M))
    wayplist.append(Waypoint(pickpoint@lift,  W-1.75*cfg.FINGER_SQUEEZE_M,1))      # 2. move onto the block
    wayplist.append(Waypoint(clear@lift, W-1.75*cfg.FINGER_SQUEEZE_M, inside_m/cfg.PULL_INSIDE_SPEED_MS))
    wayplist.append(Waypoint(pickpounce@lift, W-1.75*cfg.FINGER_SQUEEZE_M, (0.2-inside_m)/cfg.PULL_OUT_SPEED_MS))
    wayplist.append(Waypoint(pickpounce@upoffset, W-1.75*cfg.FINGER_SQUEEZE_M))
    return wayplist

def pedestal_target(cfg):
    """Where the block should end up on the pedestal on the robot's left (D-22), in the robot frame at the pull
    station: block frame B (bottom-face centre) over the post, long axis pointing at the tower, bottom 2 mm below
    the post top (pressed in slightly so it sits).

    Inputs: cfg (config).  Returns: RigidTransform X_RB.
    """
    from pydrake.all import RigidTransform, RotationMatrix
    d = cfg.PEDESTAL_DISTANCE_M / np.sqrt(2)
    return RigidTransform(RotationMatrix.MakeZRotation(-np.pi/4), [cfg.STATION_STANDOFF_M - d, d, cfg.PEDESTAL_HEIGHT_M - 0.002])


def plan_ped_place(X_RB, cfg, target=None):
    """Waypoints that carry the pulled block to the pedestal on the robot's left (D-22), set it down with its
    long axis pointing at the tower, let go and rise.  The first waypoint is a joint move, which also puts the
    arm in a configuration that can reach the pedestal; plan_regrab then picks the block up level by its ends.

    Inputs: X_RB (the block's pose after plan_pull; unused), cfg (config), target (where the block should end up,
    X_RB on the post; None = pedestal_target(cfg)).  The plan moves rigidly with the target, so the mover can
    shift it by where the block really sits in the hand (mover.grasp_corrected_target, as for the place).
    Returns: list of Waypoint: [0] 100 mm above the set point, [1] the set point (still gripping), then release
    and rise.
    """
    from pydrake.all import RigidTransform, RotationMatrix
    L,W,H = geo.block_dims(cfg)
    X_RBped = pedestal_target(cfg) if target is None else target
    R_BG = RotationMatrix.MakeXRotation(np.radians(-180))@RotationMatrix.MakeZRotation(np.radians(-90))
    X_BG = RigidTransform(R_BG,[-L/2-cfg.FINGERTIP_REACH_M+L/5,0,0.5*H])
    setpoint = X_RBped @ X_BG
    above = RigidTransform([0, 0, -0.1])
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
    wayplist.append(Waypoint(setpounce, cfg.FINGER_OPEN_COMMAND_M)) 
    wayplist.append(Waypoint(setpounce, cfg.FINGER_OPEN_COMMAND_M, 1))        # only now lift
   # wayplist.append(Waypoint(end_pinch.plan_pull(X_RBped, cfg)[0].X_RG @ above, cfg.FINGER_OPEN_COMMAND_M))
    return wayplist


def plan_regrab(X_RB, cfg):
    """Pick the block up off the pedestal by its ends (D-22): the same grasp end-pinch uses in the tower,
    with one extra first waypoint 20 mm above end-pinch's start.  That first waypoint is reached by a joint
    move whose arc passes the block on the post: 20 mm up leaves 32 mm of gripper clearance, 21.5 mm without
    it, where the arc knocked the block off (kinematic check, scratch/check_regrab_starts.py).  50 mm (66 mm)
    placed on the Mac, but on the cluster the hand stopped above the place point and let go in the air; its
    larger change of arm shape is the likely cause (not confirmed).  The base rolls to the place without the
    tuck into the carry pose (mover.RealMover, O-5)."""
    from pydrake.all import RigidTransform
    wps = end_pinch.plan_pull(X_RB, cfg)  # wps[0] is end-pinch's start (pounce) pose
    high = wps[0].X_RG @ RigidTransform([0, 0, -0.02])  # 20 mm up (end-pinch's G z points down)
    return [Waypoint(high, cfg.FINGER_OPEN_COMMAND_M)] + wps


def plan_place(X_RT, cfg):
    return end_pinch.plan_place(X_RT, cfg)
