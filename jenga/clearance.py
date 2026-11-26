"""ARM CLEARANCE: TODO for the team (DECISIONS.md, open items).

The arm model has NO collision geometry (measured from the model's file name), so the simulator will not
stop the arm passing through the table, the platform or the tower.  Nothing here checks the arm links yet.
Options the team can weigh: an iiwa model with collision geometry (check whether one ships with Drake),
geometric proxies (capsules per link) checked on each planned path, or watching the replays.

Swing clearance (below, config.CHECK_SWING_CLEARANCE, off by default): the executor's joint-space moves pick an
IK solution whose path keeps the GRIPPER and the HELD BLOCK clear of the table, platform, pedestals and blocks.
The arm links are still not checked.

Two hooks exist so the decision can be made later without restructuring:
  * check_motion() is called once per motion by the executor.  It does nothing yet.
  * ARM_COLLISIONS_CHECKED feeds the CSV column `arm_collisions_unchecked`.  Set it True ONLY when
    check_motion really checks, so a run is never mistaken for a checked one.
"""

ARM_COLLISIONS_CHECKED = False  # leave False until check_motion() below really checks


def check_motion(world, waypoints, label: str = "") -> None:
    """Called before each motion.  Raise jenga.errors.MotionFailed if the planned motion would hit something.

    Inputs: world (the Sim), waypoints (list of Waypoint in the robot frame), label (text for messages).
    """
    # ---- BEGIN CODE TO BE WRITTEN ----
    return None  # TODO: the team decides how (or whether) to check; until then nothing is checked
    # ---- END CODE TO BE WRITTEN ----


# ====================================================================== swing clearance (config.CHECK_SWING_CLEARANCE)
# A plan's first move is a straight line in joint angles to an IK solution, and WHICH solution decides where the
# hand goes on the way.  With the check on, the executor takes the solution it would have preferred anyway, unless
# its path brings the gripper or the held block within SWING_CLEARANCE_M of the table, platform, pedestals or any
# other block; then the next one in the same order, and so on.  If none is clear the motion fails (MotionFailed).
# Only the gripper and the held block are checked: in the simulator only they (not the arm links) collide (F-01),
# so ARM_COLLISIONS_CHECKED stays False.  Why: in the push sweep the pedestal swing carried the held block ~205 mm
# through the table in 4 of 10 runs and the block fell off the post (measured, from the logged joint angles).
# Everything below is plain numpy apart from world.kin, so the geometry is unit-tested without Drake.

import numpy as np  # noqa: E402

from jenga import base as base_mod  # noqa: E402
from jenga import geometry as geo  # noqa: E402
from jenga.errors import MotionFailed  # noqa: E402

GRID_M = 0.01  # [estimate] spacing of the points on the moving boxes; the clearance margin must be larger


def surface_points(dims, centre=(0.0, 0.0, 0.0), step=GRID_M) -> np.ndarray:
    """Points spread over the surface of a box, used to stand for the box in distance checks.

    Inputs: dims (box size x, y, z in m), centre (box centre in the parent frame), step (largest gap between
    neighbouring points, m).
    Returns: (n x 3) array of points in the parent frame, corners and edges included.
    """
    g = [np.linspace(-v / 2, v / 2, max(int(np.ceil(v / step)), 1) + 1) for v in dims]
    X, Y, Z = np.meshgrid(*g, indexing="ij")
    P = np.column_stack([X.ravel(), Y.ravel(), Z.ravel()])
    on = np.any(np.isclose(np.abs(P), np.asarray(dims) / 2), axis=1)
    return P[on] + np.asarray(centre, float)


def box_distance(points, X_WBox, half) -> np.ndarray:
    """Signed distance from each point to a box (positive outside, negative inside).

    Inputs: points ((n x 3) world points), X_WBox (4x4 pose of the box centre in the world), half (half sizes of
    the box along its own x, y, z, m).
    Returns: (n,) array of distances in m.
    """
    p = (np.asarray(points) - X_WBox[:3, 3]) @ X_WBox[:3, :3]
    d = np.abs(p) - np.asarray(half)
    return np.linalg.norm(np.maximum(d, 0.0), axis=1) + np.minimum(d.max(axis=1), 0.0)


def gripper_points(cfg, opening: float) -> np.ndarray:
    """Points on the gripper's collision shapes: the hand body and the two fingers (sizes from config, F-02).

    Inputs: cfg (config), opening (commanded finger opening, m; the pads sit opening + PAD_GAP_EXTRA_M apart).
    Returns: (n x 3) array of points in the gripper frame G (x finger axis, y reach axis).
    """
    hx, hy, hz = cfg.HAND_HOUSING_M
    housing = surface_points((hx, hy, hz), (0.0, cfg.HAND_HOUSING_FRONT_Y_M - hy / 2, 0.0))
    flen = cfg.FINGERTIP_REACH_M - cfg.HAND_HOUSING_FRONT_Y_M
    fx = (opening + cfg.PAD_GAP_EXTRA_M) / 2 + cfg.PAD_THICKNESS_M + cfg.FINGER_THICKNESS_M / 2  # behind the pad
    fingers = [surface_points((cfg.FINGER_THICKNESS_M, flen, cfg.FINGER_HEIGHT_M),
                              (s * fx, cfg.HAND_HOUSING_FRONT_Y_M + flen / 2, 0.0)) for s in (-1, 1)]
    return np.vstack([housing] + fingers)


def _pose(xyz, yaw=0.0) -> np.ndarray:
    """4x4 pose at xyz, turned by yaw (rad) about z."""
    X = np.eye(4)
    c, s = np.cos(yaw), np.sin(yaw)
    X[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    X[:3, 3] = xyz
    return X


def obstacles(cfg, block_poses: dict, skip=()) -> list:
    """Everything a swing must keep clear of: the table, the platform, the four pedestals and the blocks.
    The same boxes jenga/scene.py builds.

    Inputs: cfg (config), block_poses (block id -> 4x4 world pose of the block centre, from the simulator),
    skip (block ids to leave out, e.g. the block in the hand).
    Returns: list of (name, 4x4 world pose of the box centre, half sizes) tuples.
    """
    cx, cy = cfg.TOWER_CENTER_XY
    fp = geo.tower_footprint_m(cfg) + 2 * cfg.PLATFORM_MARGIN_M
    tx, ty, tz = cfg.TABLE_SIZE_M
    out = [("table", _pose((0.0, 0.0, -tz / 2)), (tx / 2, ty / 2, tz / 2)),
           ("platform", _pose((cx, cy, cfg.PLATFORM_HEIGHT_M / 2)), (fp / 2, fp / 2, cfg.PLATFORM_HEIGHT_M / 2))]
    for i, (x, y, yaw) in enumerate(base_mod.pedestal_poses(cfg)):
        out.append((f"pedestal_{i}", _pose((x, y, cfg.PEDESTAL_HEIGHT_M / 2), yaw),
                    (cfg.PEDESTAL_TOP_M / 2, cfg.PEDESTAL_TOP_M / 2, cfg.PEDESTAL_HEIGHT_M / 2)))
    half = np.asarray(geo.block_dims(cfg)) / 2
    out += [(f"block_{b}", np.asarray(X, float), half) for b, X in block_poses.items() if b not in skip]
    return out


def joint_path(q0, q1, step_rad: float) -> np.ndarray:
    """Evenly spaced arm angles along the straight line in joint space from q0 to q1 (the path of a joint move).
    The two ends are left out: they are the plan's own poses; the check is about the way between them.

    Inputs: q0, q1 (start and end arm angles, rad), step_rad (largest change of any joint between samples).
    Returns: (m x 7) array of arm angles; empty if q0 and q1 are one step apart or less.
    """
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    n = max(int(np.ceil(np.max(np.abs(q1 - q0)) / step_rad)), 1)
    return np.array([q0 + (i / n) * (q1 - q0) for i in range(1, n)])


def path_clearance(fk, X_WR, q0, q1, moving_G, obs, step_rad: float, stop_below: float = -np.inf):
    """How close the moving parts come to the obstacles along a joint move from q0 to q1.

    Inputs: fk (function: arm angles -> 4x4 gripper pose X_RG in the robot frame), X_WR (4x4 robot base pose in
    the world), q0, q1 (start and end arm angles), moving_G ((n x 3) points fixed in the gripper frame: gripper and
    held block), obs (list from obstacles()), step_rad (sampling step, see joint_path), stop_below (stop early once
    the clearance is below this, m).
    Returns: (smallest clearance in m, name of the obstacle it was to); (inf, "") for a path with no samples.
    """
    P = np.hstack([moving_G, np.ones((len(moving_G), 1))]).T
    best, what = np.inf, ""
    for q in joint_path(q0, q1, step_rad):
        p = (X_WR @ fk(q) @ P)[:3].T
        for name, X, half in obs:
            d = float(box_distance(p, X, half).min())
            if d < best:
                best, what = d, name
        if best < stop_below:
            break
    return best, what


def preference_order(solutions, q_ref, margin_fn, good_margin: float) -> list:
    """IK solutions in the order the executor prefers them (the rule of kinematics.solve_ik_best): those at least
    good_margin from every joint limit, nearest q_ref first; then the rest, largest margin first.  The first
    element is the solution solve_ik_best would return.

    Inputs: solutions (list of arm-angle arrays), q_ref (current arm angles), margin_fn (function: arm angles ->
    smallest distance to a joint limit, rad), good_margin (rad).
    Returns: the same solutions as a new list, in that order.
    """
    q_ref = np.asarray(q_ref, float)
    good = sorted((q for q in solutions if margin_fn(q) >= good_margin), key=lambda q: np.linalg.norm(q - q_ref))
    rest = sorted((q for q in solutions if margin_fn(q) < good_margin), key=margin_fn, reverse=True)
    return good + rest


def swing_poses(world, held=None) -> dict:
    """Block poses for the swing check.  Truth, or with config.PERCEPTION the perceived ones (world.perceived, set by
    perceive.observe and observe_block), and the block in the hand from the cameras too (perceive.observe_held).
    Blocks perception does not track (fallen off the tower) are left out."""
    truth = world.block_poses()
    if not world.cfg.PERCEPTION or getattr(world, "perceived", None) is None:
        return truth
    from jenga import perceive  # here: perceive needs scipy, which the quick checks may not load
    poses = dict(world.perceived)
    if held is not None:
        poses[held] = perceive.observe_held(world, world.cfg, held, label="held swing")
    return poses


def choose_swing(world, X_RG, opening: float, held=None, label: str = ""):
    """Arm angles for a joint move to the gripper pose X_RG whose path keeps the gripper (and the held block, if
    any) at least SWING_CLEARANCE_M from every obstacle.  Tries the IK solutions in preference_order and takes the
    first clear one, so it agrees with solve_ik_best whenever that solution's path is clear.

    Inputs: world (the Sim: cfg, kin, arm_q, X_WR, block_poses), X_RG (target gripper pose, robot frame),
    opening (widest finger opening during the move, m), held (id of the block in the hand, or None; it is taken
    to move rigidly with the hand from where it is now), label (text for messages).
    Returns: (arm angles, summary dict for the development log: swing_tried = position of the chosen solution in
    the order (1 = the usual choice), swing_options = how many solutions, swing_clear_mm, swing_nearest).
    Raises: MotionFailed if there is no IK solution or none of them has a clear path.
    """
    cfg, kin = world.cfg, world.kin
    q_now = world.arm_q()
    sols = kin.ik_solutions(X_RG, q_now)
    if not sols:
        raise MotionFailed(f"{label}: no IK solution for the hand pose (is it reachable from this station?)")
    order = preference_order(sols, q_now, kin.joint_limit_margin, 0.15)  # 0.15: solve_ik_best's good_margin
    poses = swing_poses(world, held)
    fk = lambda q: np.asarray(kin.gripper_pose(q).GetAsMatrix4())  # noqa: E731
    X_WR = np.asarray(world.X_WR(), float)
    X_RG_now = fk(q_now)
    moving = [gripper_points(cfg, opening)]
    if held is not None:  # the held block, rigid in the hand as it is now
        X_GB = np.linalg.inv(X_WR @ X_RG_now) @ poses[held]
        pts = surface_points(geo.block_dims(cfg))
        moving.append(pts @ X_GB[:3, :3].T + X_GB[:3, 3])
    moving = np.vstack(moving)
    obs = obstacles(cfg, poses, skip=() if held is None else (held,))
    best = (-np.inf, "")
    for i, q in enumerate(order):
        d, what = path_clearance(fk, X_WR, q_now, q, moving, obs, cfg.SWING_CHECK_STEP_RAD, cfg.SWING_CLEARANCE_M)
        if d >= cfg.SWING_CLEARANCE_M:
            return q, dict(swing_tried=i + 1, swing_options=len(order), swing_clear_mm=round(d * 1000, 1),
                           swing_nearest=what)
        best = max(best, (d, what))
    raise MotionFailed(f"{label}: none of {len(order)} IK solutions has a swing clear by "
                       f"{cfg.SWING_CLEARANCE_M * 1000:.0f} mm (best {best[0] * 1000:.1f} mm from {best[1]})")
