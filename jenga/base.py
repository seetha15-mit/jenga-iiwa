"""Where the rolling robot base can stand, and how it gets there.  Pure Python: no Drake needed.

A STATION is just a base pose in the world: the robot stands STATION_STANDOFF_M from the tower
centre and faces it.  Stations are a plain list built from config.STATION_ANGLES_DEG, and the base-move
step (jenga/executor.py: move_base_to) accepts any station index, so "a few stations" can later become
"any pose" by replacing choose_station() in a method, not the team's pinch/pull/place code.

Angle convention: station angle theta means the robot faces along (cos theta, sin theta), so it stands
at  tower_centre - standoff * (cos theta, sin theta)  with its x axis along (cos theta, sin theta).
theta = 0 deg: robot at -x of the tower, facing +x.
"""

import numpy as np

from jenga import geometry as geo


def station_angles(cfg):
    return [np.radians(a) for a in cfg.STATION_ANGLES_DEG]


def approach_dir(cfg, k: int) -> np.ndarray:
    """Unit vector (world, horizontal) the robot at station k faces."""
    th = station_angles(cfg)[k]
    return np.array([np.cos(th), np.sin(th)])


def base_xy_yaw(cfg, k: int):
    """(x, y, yaw) of the base at station k, in the world."""
    th = station_angles(cfg)[k]
    cx, cy = cfg.TOWER_CENTER_XY
    r = cfg.STATION_STANDOFF_M
    return (cx - r * np.cos(th), cy - r * np.sin(th), th)


def station_pose(cfg, k: int) -> np.ndarray:
    """4x4 pose X_WR of the robot base frame at station k (arm mounting flange)."""
    x, y, yaw = base_xy_yaw(cfg, k)
    return geo.make_pose(x, y, cfg.BASE_MOUNT_HEIGHT_M, yaw)


def to_robot_frame(X_WR: np.ndarray, X_W: np.ndarray) -> np.ndarray:
    """Express a world pose in the robot base frame.  This is what the team's steps receive."""
    return np.linalg.inv(X_WR) @ X_W


# ---------------------------------------------------------------- which station sees a block as 'near edge'
def near_edge_stations(tower, block_id: int):
    """Stations from which this block is the NEAR EDGE block of its layer, with the block's long
    axis across the approach direction (so the jaw can close on its two ends).  Usually one."""
    cfg = tower.cfg
    rec = tower.blocks[block_id]
    if rec.layer is None or rec.slot not in (0, 2):
        return []
    across = tower.across_axis(rec.layer)
    sign = -1.0 if rec.slot == 0 else 1.0  # slot 0 sits at the negative end of the across axis
    out = []
    for k in range(len(cfg.STATION_ANGLES_DEG)):
        a = approach_dir(cfg, k)
        c = float(across @ a)
        if abs(abs(c) - 1.0) > 1e-6:  # approach must be along the across axis (block length across it)
            continue
        if sign * c < 0:  # block sits on the robot's side of the tower: it is the near one
            out.append(k)
    return out


def push_stations(tower, block_id: int):
    """Stations that face this block's END (short) face, so a push along its long axis slides it out
    the far side.  Any slot works, including the middle; either end works."""
    cfg = tower.cfg
    rec = tower.blocks[block_id]
    if rec.layer is None:  # a dropped/loose block is no longer in the tower
        return []
    across = tower.across_axis(rec.layer)
    out = []
    for k in range(len(cfg.STATION_ANGLES_DEG)):
        c = float(across @ approach_dir(cfg, k))
        if abs(c) < 1e-6:  # approach perpendicular to across  ->  approach along the long axis
            out.append(k)
    return out


def nearest_station(cfg, candidates, current: int):
    """The candidate station with the smallest angle from the current one (shortest base move)."""
    ang = station_angles(cfg)

    def dist(k):
        d = (ang[k] - ang[current] + np.pi) % (2 * np.pi) - np.pi
        return abs(d)

    return min(candidates, key=dist)


# ---------------------------------------------------------------- rolling path
def _to_polar(cfg, x, y):
    """(radius, bearing) of the base about the tower centre, where bearing is the facing direction."""
    cx, cy = cfg.TOWER_CENTER_XY
    r = float(np.hypot(x - cx, y - cy))
    theta = float(np.arctan2(cy - y, cx - x))  # direction from the base toward the tower
    return r, theta


def base_path(cfg, start_xy_yaw, k_target: int):
    """Waypoints (x, y, yaw) for the base from where it is now to station k_target.

    Roll straight out to BASE_TRAVEL_RADIUS_M, around the tower along a circle (the robot keeps facing
    the tower), then straight in.  The arm is tucked meanwhile (config.ARM_CARRY_Q).  Yaw is unwrapped
    (continuous), because the base's yaw joint has no limit.

    Only 'face the tower' poses are supported here.  A free-moving base (option C in DECISIONS.md)
    would replace this function."""
    cx, cy = cfg.TOWER_CENTER_XY
    x0, y0, yaw0 = start_xy_yaw
    r0, th0 = _to_polar(cfg, x0, y0)
    r1 = cfg.STATION_STANDOFF_M
    th1_world = station_angles(cfg)[k_target]
    dth = (th1_world - th0 + np.pi) % (2 * np.pi) - np.pi  # shortest way round
    R = max(cfg.BASE_TRAVEL_RADIUS_M, r0, r1)
    yaw1 = yaw0 + dth

    def at(r, th, yaw):
        return (cx - r * np.cos(th), cy - r * np.sin(th), yaw)

    pts = [(x0, y0, yaw0), at(R, th0, yaw0)]
    n_arc = max(2, int(np.ceil(abs(dth) / np.radians(5.0))))
    for i in range(1, n_arc + 1):
        f = i / n_arc
        pts.append(at(R, th0 + f * dth, yaw0 + f * dth))
    pts.append(at(r1, th0 + dth, yaw1))
    # drop duplicate points
    out = [pts[0]]
    for p in pts[1:]:
        if np.linalg.norm(np.array(p) - np.array(out[-1])) > 1e-9:
            out.append(p)
    return out


def sample_path(points, speed: float, dt: float):
    """Targets (x, y, yaw) at every dt seconds along a polyline at constant speed.
    The last element is exactly the final point.  A pure rotation counts as 0.3 m per radian."""
    pts = [np.array(p, float) for p in points]
    lengths = [max(float(np.hypot(b[0] - a[0], b[1] - a[1])), abs(b[2] - a[2]) * 0.3)
               for a, b in zip(pts[:-1], pts[1:])]
    total = sum(lengths)
    if total <= 0:
        return [tuple(pts[-1])]
    step = speed * dt
    n = max(1, int(np.ceil(total / step)))
    ends = np.cumsum(lengths)
    out = []
    for j in range(1, n + 1):
        s = min(j * step, total)
        i = int(np.searchsorted(ends, s - 1e-12))
        i = min(i, len(lengths) - 1)
        start = ends[i] - lengths[i]
        f = (s - start) / lengths[i] if lengths[i] > 0 else 1.0
        out.append(tuple(pts[i] + f * (pts[i + 1] - pts[i])))
    out[-1] = tuple(pts[-1])
    return out
