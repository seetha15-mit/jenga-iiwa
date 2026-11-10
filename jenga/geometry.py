"""Sizes and simple geometry derived from config.py.  Pure Python: no Drake needed.

Block body frame: origin at the centre of the block's BOTTOM face, long axis = body x.
Setting that frame's z to a surface height rests the block on the surface.
"""

import numpy as np


# ---------------------------------------------------------------- poses (4x4 matrices, world frame)
def make_pose(x, y, z, yaw=0.0) -> np.ndarray:
    """4x4 transform for a position and a rotation about z (radians)."""
    c, s = np.cos(yaw), np.sin(yaw)
    X = np.eye(4)
    X[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    X[:3, 3] = [x, y, z]
    return X


def yaw_of(X: np.ndarray) -> float:
    return float(np.arctan2(X[1, 0], X[0, 0]))


def quat_wxyz(R: np.ndarray) -> np.ndarray:
    """Unit quaternion [w, x, y, z] of a 3x3 rotation matrix."""
    R = np.asarray(R, float)
    t = np.trace(R)
    if t > 0:
        s = 2.0 * np.sqrt(t + 1.0)
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(R[i, i] - R[j, j] - R[k, k] + 1.0)
        v = [0.0, 0.0, 0.0]
        v[i] = 0.25 * s
        v[j] = (R[j, i] + R[i, j]) / s
        v[k] = (R[k, i] + R[i, k]) / s
        w = (R[k, j] - R[j, k]) / s
        q = [w, *v]
    q = np.array(q)
    return q / np.linalg.norm(q)


def free_body_q(X: np.ndarray) -> np.ndarray:
    """Drake's 7 positions for a free body: [qw, qx, qy, qz, x, y, z] (what SetDefaultPositions / SetPositions take)."""
    return np.concatenate([quat_wxyz(X[:3, :3]), np.asarray(X, float)[:3, 3]])


# ---------------------------------------------------------------- blocks
def block_dims(cfg):
    """(length, width, height) in meters: real size times ONE scale factor."""
    return tuple(v * cfg.BLOCK_SCALE for v in cfg.BLOCK_REAL_M)


def block_mass(cfg) -> float:
    L, W, H = block_dims(cfg)
    return cfg.BLOCK_DENSITY * L * W * H


def assign_friction(cfg, seed: int) -> dict:
    """HIDDEN ground truth {block_id: mu}.  Only the simulator and the evaluation side may read it."""
    n = 3 * cfg.N_LAYERS_START
    if cfg.FRICTION_MODE == "global":
        return {i: float(cfg.FRICTION_GLOBAL) for i in range(n)}
    if cfg.FRICTION_MODE == "random":
        rng = np.random.default_rng(10_000 + seed)  # separate stream so it never shifts the pick stream
        lo, hi = cfg.FRICTION_RANGE
        return {i: float(rng.uniform(lo, hi)) for i in range(n)}
    raise ValueError(f"FRICTION_MODE must be 'global' or 'random', got {cfg.FRICTION_MODE!r}")


def tower_across_m(cfg) -> float:
    """Width of the tower across a layer: three blocks side by side plus two gaps."""
    _, W, _ = block_dims(cfg)
    return 3 * W + 2 * cfg.BLOCK_GAP_M


def tower_footprint_m(cfg) -> float:
    """Side of the (square) tower footprint: the larger of block length and the across width."""
    L, _, _ = block_dims(cfg)
    return max(L, tower_across_m(cfg))


# ---------------------------------------------------------------- gripper fit
def pad_gap_m(cfg) -> float:
    """Gap between the finger pads at the fully open command [measured: command + 5 mm]."""
    return cfg.FINGER_OPEN_COMMAND_M + cfg.PAD_GAP_EXTRA_M


def jaw_margin_per_side_m(cfg) -> float:
    """Room on each side of the block's length when the jaw is fully open (derived)."""
    L, _, _ = block_dims(cfg)
    return (pad_gap_m(cfg) - L) / 2


def finger_height_margin_per_side_m(cfg) -> float:
    """Room above and below the 20 mm fingers inside the block's height (derived).
    This is the tight one: the hand must be within this much of the right height or the pads
    touch the layers above or below the grasped block."""
    _, _, H = block_dims(cfg)
    return (H - cfg.FINGER_HEIGHT_M) / 2


def housing_overlap(cfg, slot_offset_m: float):
    """Does the hand body hit the tower when a block is placed at a slot? (derived, level hand)

    slot_offset_m: where the block centre is along the robot's reach direction, measured from the
        tower centre (negative = the near side).  For a slot with across-offset a and a station whose
        approach direction makes cosine c with the across axis, that is a * c.

    The level hand's body is 50 mm tall and sits behind the pads.  Its front face is
    (PAD_CENTRE_FROM_G - HAND_HOUSING_FRONT_Y) behind the pad centre.  Held at the top of the tower,
    the body reaches below the block's bottom plane by (50 mm - H) / 2, i.e. into the layer below.

    Returns (overlap_m, dip_m): how far the hand body's front face reaches over the tower's near
    face (<= 0 means it does not), and how far the body dips below the block's bottom plane.
    """
    _, _, H = block_dims(cfg)
    front_behind_pad = cfg.PAD_CENTRE_FROM_G_M - cfg.HAND_HOUSING_FRONT_Y_M
    front_x = slot_offset_m - front_behind_pad  # housing front face, relative to tower centre
    near_face_x = -tower_across_m(cfg) / 2
    overlap = front_x - near_face_x + cfg.HOUSING_CLEARANCE_M
    dip = cfg.HAND_HOUSING_M[2] / 2 - H / 2
    return overlap, dip


def housing_collides(cfg, slot_offset_m: float) -> bool:
    overlap, dip = housing_overlap(cfg, slot_offset_m)
    return overlap > 0 and dip > 0


# "colored": one colour per slot, a different set of three on odd layers, so every pair of touching blocks
# differs (slot neighbours touch; each block crosses all three of the layers above and below).  Hues 60 deg apart.
COLORED_EVEN = ((0.85, 0.15, 0.15), (0.15, 0.70, 0.20), (0.15, 0.30, 0.85))  # red, green, blue
COLORED_ODD = ((0.90, 0.80, 0.10), (0.10, 0.75, 0.80), (0.80, 0.20, 0.75))   # yellow, cyan, magenta
WOOD = ((0.80, 0.62, 0.40), (0.68, 0.50, 0.30))                               # light, dark by layer


def block_color(cfg, block_id: int) -> tuple:
    """RGB of a block from its STARTING layer and slot (block_id = 3 * layer + slot), for any number of layers.
    cfg.BLOCK_COLORS: "colored" (see above, to help a future perception strategy) or "wood" (two tones by layer).
    The colour stays with the block when it is moved, so the pattern holds only for the starting tower."""
    layer, slot = divmod(block_id, 3)
    if cfg.BLOCK_COLORS == "wood":
        return WOOD[layer % 2]
    return (COLORED_EVEN if layer % 2 == 0 else COLORED_ODD)[slot]
