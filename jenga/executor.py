"""The executor: plays a plan of waypoints on the persistent simulator.  (Drake code: UNTESTED here.)

A plan is a list of jenga.waypoint.Waypoint in the ROBOT frame.  `run_plan` executes it:
  * the FIRST waypoint is reached by a joint-space move (IK + a straight line in joint angles).  That is safe
    for big moves; it is not a straight line in space, so make the first waypoint a pose that is clear of the
    tower and reachable without sweeping through it.
  * every later waypoint is reached by a straight line in space (the pset's controller): good for short,
    precise moves such as the final approach, the pull and the placement.

Warnings printed (never silent): the hand ended far from its target, or an arm joint ended near its limit
(the pset controller ignores joint limits).  Arm collisions are NOT checked (jenga/clearance.py).

Every function here takes `world` = jenga.scene.Sim.
"""

from dataclasses import dataclass

import numpy as np

from jenga import base as base_mod
from jenga import clearance
from jenga.controller import as_transform, timed_trajectories
from jenga.errors import MotionFailed
from jenga.waypoint import Waypoint


@dataclass
class PhaseResult:
    label: str
    duration_s: float
    tracking_error_m: float = 0.0
    joint_margin_rad: float = 0.0


def _resolve(cfg, waypoints):
    """Fill in default openings and convert poses to RigidTransform."""
    return [
        Waypoint(as_transform(w.X_RG), cfg.FINGER_OPEN_COMMAND_M if w.opening is None else float(w.opening))
        for w in waypoints
    ]


def _check_after(world, label, target_X=None) -> PhaseResult:
    cfg = world.cfg
    q = world.arm_q()
    margin = world.kin.joint_limit_margin(q)
    err = 0.0
    if target_X is not None:
        X_now = world.kin.gripper_pose(q)
        err = float(np.linalg.norm(X_now.translation() - target_X.translation()))
        if err > cfg.TRACKING_WARN_M:
            print(f"[warn] {label or 'motion'}: the hand ended {err * 1000:.1f} mm from its target.")
    if margin < cfg.JOINT_LIMIT_WARN_RAD:
        print(f"[warn] {label or 'motion'}: an arm joint is within {margin:.3f} rad of its limit.")
    return PhaseResult(label, 0.0, err, margin)


def run_waypoints(world, waypoints, speed: float, label: str = "") -> PhaseResult:
    """Straight-line (Cartesian) motion through the waypoints, starting from where the hand is now."""
    cfg = world.cfg
    wps = _resolve(cfg, waypoints)
    clearance.check_motion(world, wps, label)
    q0 = world.arm_q()
    X_start = world.kin.gripper_pose(q0)
    v_traj, f_traj = timed_trajectories(wps, X_start, world.player.last_opening, speed)
    world.start_cartesian(v_traj, f_traj, q0)
    duration = v_traj.end_time() + cfg.HOLD_AFTER_MOTION_S
    world.advance_logged(duration)
    world.player.stop(wps[-1].opening)
    res = _check_after(world, label, wps[-1].X_RG)
    res.duration_s = duration
    return res


def move_arm_to_q(world, q_target, opening=None, label: str = "joint move") -> PhaseResult:
    """Joint-space move to the arm angles q_target (a straight line in joint angles, bell-shaped speed)."""
    cfg = world.cfg
    q0 = world.arm_q()
    q1 = np.asarray(q_target, float)
    T = max(float(np.max(np.abs(q1 - q0))) / cfg.JOINT_SPEED_RAD_S, 0.5)
    opening = world.player.last_opening if opening is None else float(opening)
    world.start_joint(q0, q1, T, opening)
    world.advance_logged(T + cfg.HOLD_AFTER_MOTION_S)
    world.player.stop(opening)
    err = float(np.max(np.abs(world.arm_q() - q1)))
    if err > 0.02:
        print(f"[warn] {label}: a joint ended {np.degrees(err):.1f} degrees from its target.")
    res = _check_after(world, label)
    res.duration_s = T + cfg.HOLD_AFTER_MOTION_S
    return res


def move_arm_to_pose(world, X_RG, opening=None, label: str = "joint move to pose") -> PhaseResult:
    """Joint-space move that puts the hand at X_RG (robot frame).  Raises MotionFailed if there is no IK solution."""
    X = as_transform(X_RG)
    q = world.kin.solve_ik_best(X, world.arm_q())
    if q is None:
        raise MotionFailed(f"{label}: no IK solution for the hand pose (is it reachable from this station?)")
    return move_arm_to_q(world, q, opening, label)


def run_plan(world, waypoints, speed: float, label: str = "") -> PhaseResult:
    """Execute a plan from the team's steps: first waypoint by a joint-space move, the rest in straight lines."""
    if not waypoints:
        raise MotionFailed(f"{label}: the plan has no waypoints")
    wps = _resolve(world.cfg, waypoints)
    clearance.check_motion(world, wps, label)
    first = move_arm_to_pose(world, wps[0].X_RG, wps[0].opening, label=f"{label}: first waypoint")
    if len(wps) == 1:
        return first
    return run_waypoints(world, wps[1:], speed, label=label)


def hold(world, seconds: float) -> None:
    world.hold(seconds)


def move_base_to_station(world, k: int) -> None:
    """Roll the base to station k (arm tucked in its carry pose, fingers keep their last command).
    Does nothing if the base is already there.  With BASE_MODE = "fixed" only station 0 exists."""
    cfg = world.cfg
    if k == world.station:
        return
    if not world.mobile:
        raise MotionFailed(f"BASE_MODE is 'fixed': cannot move from station {world.station} to {k}")
    move_arm_to_q(world, cfg.ARM_CARRY_Q, label="carry pose")
    path = base_mod.base_path(cfg, world.base_q(), k)
    dt = 0.05
    for target in base_mod.sample_path(path, cfg.BASE_SPEED_MS, dt):
        world.set_base_target(*target)
        world.advance(dt)
    world.set_base_target(*path[-1])
    world.hold(0.5)  # settle
    x, y, yaw = world.base_q()
    ex, ey, eyaw = path[-1]
    err = float(np.hypot(x - ex, y - ey))
    if err > 0.005 or abs(yaw - eyaw) > np.radians(1.0):
        print(f"[warn] base move to station {k}: ended {err * 1000:.1f} mm and "
              f"{np.degrees(abs(yaw - eyaw)):.1f} deg from the target.")
    world.station = k
