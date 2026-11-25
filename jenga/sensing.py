"""SENSING: notes for the team, and the perception steps the team writes (bottom of the file).  The person who
owns the force and perception strategies decides the interface, the weight compensation and the camera setup.

Facts (see DECISIONS.md for sources)
  * Wrist force: the simulator can report the reaction at the weld between the arm and the gripper
    (Sim.wrist_force_raw(), below).  The reading is in the GRIPPER frame G (measured: the hand's weight
    shows up on G y), not in the world frame.
  * The arm links have gravity switched off (an ideal gravity-compensated arm); the gripper keeps its weight.
  * It includes the hand's own weight: |F| = 10.7 N at rest with the hand pointing down (measured).  As the
    hand rotates, that weight moves between axes, so a strategy that wants the force from a block must
    deal with it.  Nothing here subtracts it.
  * Per-block friction and exact contact forces are HIDDEN from every strategy (requirement R-05).  Never read them.
  * The reading is polled between short AdvanceTo calls (jenga.scene.Sim.advance_logged), not wired as a
    port, because the hardware station does not export the plant's reaction-force port.
  * Set config.ENABLE_WRIST_FORCE_LOG = True to write a raw force trace per trial next to the replay.
  * Cameras: two RGB-D cameras, "base" (turns with the robot) and "top" (config ENABLE_CAMERAS and CAMERA_*;
    scene.Sim.camera_images).  The TA's advice: use several cameras to reduce partial observability; perception is
    harder than expected.
  * Probing with the force strategy needs motions: how a strategy commands them is open (the executor
    plays fixed waypoint lists, so it cannot stop the instant a force threshold is crossed; if the force
    strategy needs that, a stop condition would be added to jenga/executor.py without changing the steps).
"""

import numpy as np


def read_wrist_force_raw(world) -> np.ndarray:
    """[Fx, Fy, Fz, Tx, Ty, Tz] in the gripper frame, raw (includes the hand's weight, no noise)."""
    return world.wrist_force_raw()


# ====================================================================== perception (stage 2): THE TEAM WRITES THESE
# Pipeline: camera images -> world points (jenga.pointcloud, written) -> points per block by colour -> pose per block
# by ICP against a box model.  scratch/check_perception.py compares the result with the simulator's truth.
# Inputs come only from scene.Sim.camera_images(): rgb, depth, K, X_WC.  Never the label image (it is truth).

def block_model_points(cfg, spacing_m: float = 0.002) -> np.ndarray:
    """(M, 3) points spread over the SURFACE of one block, in the block's body frame B (origin at the centre of
    the BOTTOM face, x = long axis; geometry.block_dims gives L, W, H).  The model that ICP matches the camera
    points to.  Things to decide: the spacing; whether to keep the bottom face (a camera never sees it)."""
    # ---- BEGIN CODE TO BE WRITTEN ----
    raise NotImplementedError("block_model_points: CODE TO BE WRITTEN")
    # ---- END CODE TO BE WRITTEN ----


def segment_blocks(points_W: np.ndarray, rgb: np.ndarray, cfg) -> dict:
    """Split the camera points into blocks by colour.  points_W (N, 3), rgb (N, 3) in 0..1 (from
    pointcloud.camera_points_world, one or several cameras stacked).  Returns block id -> (M, 3) points.
    Each block's colour is geometry.block_color(cfg, block_id) (cfg.BLOCK_COLORS = "unique": one hue per block).
    Things to decide: compare by hue (shading changes brightness and saturation, not hue: scratch/palette_test.py);
    how to drop non-block pixels (table, robot, platform, sky: low saturation); what to do with edge pixels that
    blend two blocks (about 1% of each block)."""
    # ---- BEGIN CODE TO BE WRITTEN ----
    raise NotImplementedError("segment_blocks: CODE TO BE WRITTEN")
    # ---- END CODE TO BE WRITTEN ----


def icp_pose(scene_points: np.ndarray, model_points: np.ndarray, X_init: np.ndarray, iterations: int = 30) -> np.ndarray:
    """ICP, as in the class's geometric pose estimation: find the 4x4 world pose X of the block such that the model
    points moved by X lie on the scene points.  Start at X_init (for example the block's last known pose).
    Each iteration: (1) for each scene point, the nearest model point under the current X (scipy.spatial.cKDTree);
    (2) the best rigid transform for those pairs (centre both sets, SVD of the cross-covariance, fix a reflection);
    (3) update X.  Things to decide: point-to-point or point-to-plane; when to stop; whether to allow only a turn
    about vertical (blocks lie flat) or a full 3-D rotation; how far a match may be before it is ignored."""
    # ---- BEGIN CODE TO BE WRITTEN ----
    raise NotImplementedError("icp_pose: CODE TO BE WRITTEN")
    # ---- END CODE TO BE WRITTEN ----


def estimate_block_poses(images: dict, cfg, guesses: dict) -> dict:
    """The perception step.  images: scene.Sim.camera_images() (name -> rgb, depth, K, X_WC; ignore "label").
    guesses: block id -> 4x4 world pose to start ICP from (the tower's bookkeeping).  Returns block id -> 4x4
    estimated world pose, for the blocks it could see.  Things to decide: which cameras to use; how few points
    is too few to trust; what to return for a block nobody sees."""
    # ---- BEGIN CODE TO BE WRITTEN ----
    raise NotImplementedError("estimate_block_poses: CODE TO BE WRITTEN")
    # ---- END CODE TO BE WRITTEN ----
