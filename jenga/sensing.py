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
from jenga import geometry as geo
from scipy.spatial import cKDTree

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
    L,W,H = geo.block_dims(cfg)
    nx = int(np.ceil(L / spacing_m)) + 1 
    ny = int(np.ceil(W / spacing_m)) + 1 
    nz = int(np.ceil(H / spacing_m)) + 1 
    xpoints = np.linspace(-L/2,L/2,nx)
    ypoints = np.linspace(-W/2,W/2,ny)
    zpoints = np.linspace(0,H,nz)

    XY_x, XY_y = np.meshgrid(xpoints,ypoints)
    XZ_x, XZ_z = np.meshgrid(xpoints,zpoints)
    YZ_y,YZ_z = np.meshgrid(ypoints,zpoints)
    top = np.stack([XY_x.ravel(),XY_y.ravel(),np.full(XY_x.size,H)],axis=1)
    #no camera sees the bottom, but ICP matches scene points to model points, so unseen model points rarely get used and keeping them is harmless.
    bottom = np.stack([XY_x.ravel(),XY_y.ravel(),np.full(XY_x.size,0.0)],axis=1)
    leftside = np.stack([XZ_x.ravel(),np.full(XZ_x.size,-W/2),XZ_z.ravel()],axis=1)
    rightside = np.stack([XZ_x.ravel(),np.full(XZ_x.size,W/2),XZ_z.ravel()],axis=1)
    leftend = np.stack([np.full(YZ_y.size,-L/2),YZ_y.ravel(), YZ_z.ravel()],axis=1)
    rightend = np.stack([np.full(YZ_y.size,L/2),YZ_y.ravel(), YZ_z.ravel()],axis=1)
    return np.vstack([top,bottom,leftside,rightside,leftend,rightend])



def segment_blocks(points_W: np.ndarray, rgb: np.ndarray, cfg) -> dict:
    """Split the camera points into blocks by colour.  points_W (N, 3), rgb (N, 3) in 0..1 (from
    pointcloud.camera_points_world, one or several cameras stacked).  Returns block id -> (M, 3) points.
    Each block's colour is geometry.block_color(cfg, block_id) (cfg.BLOCK_COLORS = "unique": one hue per block).
    Things to decide: compare by hue (shading changes brightness and saturation, not hue: scratch/palette_test.py);
    how to drop non-block pixels (table, robot, platform, sky: low saturation); what to do with edge pixels that
    blend two blocks (about 1% of each block)."""
    # ---- BEGIN CODE TO BE WRITTEN ----
    margin = 0.05
    L, W, H = geo.block_dims(cfg)
    cx,cy = cfg.TOWER_CENTER_XY
    half = 0.5*geo.tower_footprint_m(cfg)+margin
    z_low = cfg.PLATFORM_HEIGHT_M+0.0001
    z_high = cfg.PLATFORM_HEIGHT_M + 10 * (H + cfg.BLOCK_GAP_M) + margin
    lower = np.array([cx-half,cy-half,z_low])
    upper = np.array([cx+half,cy+half,z_high])
    inside = np.all((points_W >= lower) & (points_W <= upper), axis=1)


    from matplotlib.colors import rgb_to_hsv
    hsv = rgb_to_hsv(rgb)
    keep = inside & (hsv[:, 1] > 0.3)
    pts = points_W[keep]
    hue = hsv[keep, 0]

    ids = range(3 * cfg.N_LAYERS_START)
    block_hue = []
    for b_id in ids:
        bcolorrgb = np.array(geo.block_color(cfg,b_id))
        block_hue.append(rgb_to_hsv(bcolorrgb)[0])
    block_hue = np.array(block_hue)
    dist = (hue[:, None] - block_hue[None, :] + 0.5) % 1 - 0.5
    nearest = np.abs(dist).argmin(axis=1)
    return {b: pts[nearest == i] for i, b in enumerate(ids)}
    # ---- END CODE TO BE WRITTEN ----


def icp_pose(scene_points: np.ndarray, model_points: np.ndarray, X_init: np.ndarray, iterations: int = 100) -> np.ndarray:
    """ICP, as in the class's geometric pose estimation: find the 4x4 world pose X of the block such that the model
    points moved by X lie on the scene points.  Start at X_init (for example the block's last known pose).
    Each iteration: (1) for each scene point, the nearest model point under the current X (scipy.spatial.cKDTree);
    (2) the best rigid transform for those pairs (centre both sets, SVD of the cross-covariance, fix a reflection);
    (3) update X.  Things to decide: point-to-point or point-to-plane; when to stop; whether to allow only a turn
    about vertical (blocks lie flat) or a full 3-D rotation; how far a match may be before it is ignored."""
    # ---- BEGIN CODE TO BE WRITTEN ----
    #bring scene into current block frame
    R_WB = X_init[:3,:3]
    p_WB = X_init[:3,3]
    tree = cKDTree(model_points)
    idx_prev = None
    for i in range(iterations):
        scene_B = (scene_points-p_WB)@R_WB # each row: R^T (p - t)
        dist, idx = tree.query(scene_B)
        if np.array_equal(idx, idx_prev):
          break
        matched = model_points[idx]
        SVDinpmat = (scene_points-scene_points.mean(axis=0)).transpose()@ (matched-matched.mean(axis=0))
        svdU, svdS, svdVt = np.linalg.svd(SVDinpmat)
        Diagsignflipper = np.diag([1.0, 1.0, np.linalg.det(svdU @ svdVt)])
        R_WB = svdU @ Diagsignflipper @ svdVt
        p_WB = scene_points.mean(axis=0)-R_WB@matched.mean(axis=0)
        idx_prev = idx
    new_X = np.eye(4)
    new_X[:3, :3] = R_WB
    new_X[:3, 3] = p_WB
    return new_X
    # ---- END CODE TO BE WRITTEN ----



def estimate_block_poses(images: dict, cfg, guesses: dict) -> dict:
    """The perception step.  images: scene.Sim.camera_images() (name -> rgb, depth, K, X_WC; ignore "label").
    guesses: block id -> 4x4 world pose to start ICP from (the tower's bookkeeping).  Returns block id -> 4x4
    estimated world pose, for the blocks it could see.  Things to decide: which cameras to use; how few points
    is too few to trust; what to return for a block nobody sees."""
    # ---- BEGIN CODE TO BE WRITTEN ----
    from jenga import pointcloud
    model = block_model_points(cfg)
    allpoints = []
    allrgb = []
    out = {}
    L, W, H = geo.block_dims(cfg)
    for im in images.values():
      points, rgb, pixels = pointcloud.camera_points_world(im)
      allpoints.append(points)
      allrgb.append(rgb)
    allpoints = np.vstack(allpoints)
    allrgb = np.vstack(allrgb)
    seg = segment_blocks(allpoints, allrgb, cfg)
    for block in guesses:
      pts = seg.get(block, np.zeros((0, 3)))
      X_WB = guesses[block]
      bfpoints = (pts - X_WB[:3,3])@X_WB[:3,:3]
      m=0.01
      near = (np.abs(bfpoints[:, 0]) < L/2 + m) & (np.abs(bfpoints[:, 1]) < W/2 + m) & (bfpoints[:, 2] > -m) & (bfpoints[:, 2] < H + m)
      pts = pts[near]
      if len(pts)<50:
        continue
      out[block] = icp_pose(pts, model, guesses[block])
    return out
    # ---- END CODE TO BE WRITTEN ----
