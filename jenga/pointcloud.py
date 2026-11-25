"""Camera images -> 3-D points in the world.  Pure Python (numpy only): no Drake needed.

The same pinhole maths as ps4's normal_estimation.project_depth_to_pC (and Drake's DepthImageToPointCloud): a pixel
(v, u) with depth Z is the point (X, Y, Z) in the camera frame C with
    X = (u - cx) Z / fx,   Y = (v - cy) Z / fy,
then p_W = X_WC p_C moves it into the world.  Drake's camera frame: z along the view, x to the image's right, y down.
"""

import numpy as np


def depth_to_camera_points(depth: np.ndarray, K: np.ndarray) -> np.ndarray:
    """(H, W) depth image (m) -> (H, W, 3) points in the camera frame.  K = 3x3 intrinsics [[fx,0,cx],[0,fy,cy],[0,0,1]].
    Pixels with no depth (inf or nan) give non-finite points; camera_points_world drops them."""
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    v, u = np.indices(depth.shape)  # image arrays index (row v, column u), as in ps4
    Z = depth.astype(float)
    return np.stack([(u - cx) * Z / fx, (v - cy) * Z / fy, Z], axis=-1)


def camera_points_world(image: dict, max_range_m: float = 3.0):
    """One camera's image dict (from scene.Sim.camera_images) -> (points_W (N, 3), rgb (N, 3) in 0..1, pixels (N, 2)
    as (v, u)), for every pixel with a depth closer than max_range_m.  Only rgb, depth, K and X_WC are read: never
    the label image (it is truth)."""
    p_C = depth_to_camera_points(image["depth"], image["K"])
    keep = np.isfinite(image["depth"]) & (image["depth"] < max_range_m)
    X_WC = np.asarray(image["X_WC"], dtype=float)
    p_W = p_C[keep] @ X_WC[:3, :3].T + X_WC[:3, 3]
    rgb = image["rgb"][keep].astype(float) / 255.0
    return p_W, rgb, np.argwhere(keep)
