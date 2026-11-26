"""Movers: what actually carries out one turn (pull a block, place it on top).

FakeMover  teleports the block to its target slot.  No arm, no physics for the move.  Exists only so the
           harness, logging, collapse detector and replay can be tested before the team's steps exist.
           Everything it produces is tagged mover = "fake" and goes to runs_fake/.  RESULTS ARE INVALID.
RealMover  runs the team's steps: rolls the base to a station, executes the method's plan_pull, rolls to a
           placing station if needed, and executes plan_place.  A method with a plan_push (the push method)
           instead runs push, pull from the opposite station, regrab, then place.

Both take a `world` (jenga.scene.Sim, or a stand-in with the same few methods in the tests).
"""

from dataclasses import dataclass

import numpy as np

from jenga import base, clearance, diaglog, executor_api  # noqa: F401  (executor_api: lazy Drake import point)
from jenga import methods


def _mat(X) -> np.ndarray:
    return np.asarray(X.GetAsMatrix4() if hasattr(X, "GetAsMatrix4") else X, dtype=float)


def release_index(waypoints, cfg):
    """Index of the first waypoint whose opening is wider than the one before: the hand's release pose.
    None if the fingers never open."""
    widths = [cfg.FINGER_OPEN_COMMAND_M if w.opening is None else w.opening for w in waypoints]
    return next((i for i in range(1, len(widths)) if widths[i] > widths[i - 1]), None)


def release_target(tower, layer, slot, poses, cfg, held=None) -> np.ndarray:
    """The slot's pose with its height set to the MEASURED top of the layer below (mean bottom face of the blocks
    there + one block height), not the built height: the tower is built with a gap between layers and settles, so
    the built height sits above the real top by more each layer (D-25).  poses: block id -> 4x4 world pose
    (simulator truth, or the perceived tower with config.PERCEPTION, stage 3).  held: the block in
    the hand, never counted."""
    X = np.array(tower.target_pose(layer, slot), dtype=float)
    below = [b for b in tower.layer_blocks(layer - 1) if b != held] if layer > 0 else []
    if below:
        X[2, 3] = np.mean([poses[b][2, 3] for b in below]) + tower.dims[2]
    return X


def grasp_corrected_target(X_RT, X_RG_release, X_RG_now, X_RB_now) -> np.ndarray:
    """The place target to hand plan_place so the block lands at X_RT although it sits off-centre in the hand.

    plan_place assumes the block is at X_GB = inv(X_RG_release) X_RT in the hand; it is really at
    inv(X_RG_now) X_RB_now.  Moving the target by minus that offset (turned into the robot frame at the
    release) moves the hand, and so the block, back onto the slot.  Translation only: a tilt in the hand is not
    corrected.  All 4x4 robot-frame poses; plan_place must move rigidly with its target (end_pinch's does)."""
    X_RT, X_RG_release = _mat(X_RT), _mat(X_RG_release)
    assumed = (np.linalg.inv(X_RG_release) @ X_RT)[:3, 3]
    actual = (np.linalg.inv(_mat(X_RG_now)) @ _mat(X_RB_now))[:3, 3]
    out = X_RT.copy()
    out[:3, 3] -= X_RG_release[:3, :3] @ (actual - assumed)
    return out


@dataclass
class TurnInfo:
    station_pull: int = -1
    station_place: int = -1


class FakeMover:
    TAG = "fake"

    def banner(self) -> str:
        bar = "*" * 78
        return (f"{bar}\n*** FAKE MOVER: blocks are TELEPORTED.  Results are INVALID and are written to "
                f"the fake-runs folder. ***\n{bar}")

    def do_turn(self, world, tower, block_id, target, cfg) -> TurnInfo:
        layer, slot = target
        world.teleport_block(block_id, tower.target_pose(layer, slot))
        world.hold(1.0)  # let the tower settle so a disturbed neighbour shows up in the poses
        return TurnInfo()


class RealMover:
    TAG = "real"
    last_diag = None  # the last turn's development log (jenga/diaglog.py), or None when it is off

    def banner(self) -> str:
        return ""

    def do_turn(self, world, tower, block_id, target, cfg) -> TurnInfo:
        from pydrake.all import RigidTransform  # Drake only needed on the real path

        ex = executor_api.get()
        method = methods.get_method(cfg.EXTRACTION_METHOD)
        if hasattr(method, "method_for"):  # mixed: this block's own method runs the whole turn (D-07)
            method = method.method_for(tower, block_id)
        layer, slot = target

        def in_R(X_WB):  # a world pose in the robot frame, where the base stands NOW
            return RigidTransform(base.to_robot_frame(world.X_WR(), X_WB))

        dlog = diaglog.StepLog(world, tower, block_id, cfg) if cfg.DIAG_LOG_LEVEL > 0 else None
        self.last_diag = dlog  # the harness writes its rows (steps.csv), also after a failed motion

        def do(name, fn, *args, **kw):  # run one step; with the development log on, record what it did
            if dlog:
                dlog.begin_step()
                if dlog.per_waypoint and fn in (ex.run_plan, ex.run_waypoints):
                    kw["on_waypoint"] = lambda i, wp: dlog.waypoint(name, i, wp)
            try:
                res = fn(*args, **kw)
            except Exception as e:
                if dlog:
                    dlog.step(name, error=str(e))
                raise
            if dlog:
                dlog.step(name, res)
            return res

        if hasattr(method, "plan_push"):
            # ---- push (push method): from one end, the block slides part way out the far side
            do("roll_to_push", ex.move_base_to_station, world, method.choose_station(tower, block_id, cfg, world.station))
            do("push", ex.run_plan, world, method.plan_push(in_R(tower.block_pose(block_id)), cfg), cfg.WAYPOINT_SPEED_MS,
               label="push")
            # ---- pull from the far end, then regrab; the block has moved, so its pose is read again
            k_pull = method.choose_pull_station(tower, block_id, cfg, world.station)
            do("roll_to_pull", ex.move_base_to_station, world, k_pull)
            do("pull", ex.run_plan, world, method.plan_pull(in_R(world.block_poses()[block_id]), cfg),
               cfg.WAYPOINT_SPEED_MS, label="pull")
            # ---- set it on a pedestal, then regrab it from where it actually landed (D-22); each is its own plan,
            # so each starts with a joint move (this clears the hand's drift before the regrab grasp)
            # swing to above the post; then read where the block really sits in the hand (it slides 6-15 mm down
            # the fingers during the swing) and shift the set-down by that, as for the place (D-08); then go down
            ped = method.plan_ped_place(in_R(world.block_poses()[block_id]), cfg)
            do("pedestal", ex.run_plan, world, ped[:1], cfg.WAYPOINT_SPEED_MS, label="pedestal", held=block_id)
            if cfg.PEDESTAL_CORRECT_GRASP:
                X_T = RigidTransform(grasp_corrected_target(
                    method.pedestal_target(cfg), ped[1].X_RG, world.kin.gripper_pose(world.arm_q()),
                    base.to_robot_frame(world.X_WR(), world.block_poses()[block_id])))
                ped = method.plan_ped_place(in_R(world.block_poses()[block_id]), cfg, target=X_T)
            do("pedestal_set", ex.run_waypoints, world, ped, cfg.WAYPOINT_SPEED_MS, label="pedestal set-down",
               first_index=0)
            do("regrab", ex.run_plan, world, method.plan_regrab(in_R(world.block_poses()[block_id]), cfg),
               cfg.WAYPOINT_SPEED_MS, label="regrab")
        else:
            # ---- pull
            k_pull = method.choose_station(tower, block_id, cfg, world.station)
            do("roll_to_pull", ex.move_base_to_station, world, k_pull)
            do("pull", ex.run_plan, world, method.plan_pull(in_R(tower.block_pose(block_id)), cfg),
               cfg.WAYPOINT_SPEED_MS, label="pull")

        # ---- place (the block stays in the hand while the base rolls)
        k_place = method.choose_place_station(tower, layer, slot, cfg, world.station)
        # no tuck before the place: the arm resets before each pull/push (O-5)
        do("roll_to_place", ex.move_base_to_station, world, k_place, tuck=False)
        poses = {b: tower.block_pose(b) for b in tower.blocks} if cfg.PERCEPTION else world.block_poses()  # perceived or truth
        X_RT = in_R(release_target(tower, layer, slot, poses, cfg, held=block_id))
        wps = method.plan_place(X_RT, cfg)
        i = release_index(wps, cfg) if cfg.PLACE_CORRECT_GRASP else None
        if i is not None:  # aim the block, not the hand: it slides in the pads during the roll (D-08)
            X_RT = RigidTransform(grasp_corrected_target(
                X_RT, wps[i].X_RG, world.kin.gripper_pose(world.arm_q()),
                base.to_robot_frame(world.X_WR(), world.block_poses()[block_id])))
            wps = method.plan_place(X_RT, cfg)
        do("place", ex.run_plan, world, wps, cfg.WAYPOINT_SPEED_MS, label="place", held=block_id)
        return TurnInfo(k_pull, k_place)


def get_mover(name: str):
    if name == "fake":
        return FakeMover()
    if name == "real":
        return RealMover()
    raise ValueError(f"MOVER must be 'fake' or 'real', got {name!r}")
