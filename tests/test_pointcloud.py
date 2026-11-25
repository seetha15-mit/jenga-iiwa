"""pointcloud: depth pixels -> camera points -> world points.  Pure Python."""

import unittest

import numpy as np

from jenga import geometry as geo
from jenga import pointcloud as pc

K = np.array([[500.0, 0, 320], [0, 500.0, 240], [0, 0, 1]])


class TestPointcloud(unittest.TestCase):
    def test_centre_pixel_is_on_the_axis(self):
        depth = np.full((480, 640), 2.0)
        p = pc.depth_to_camera_points(depth, K)
        np.testing.assert_allclose(p[240, 320], [0, 0, 2.0])

    def test_pinhole_offsets(self):
        depth = np.full((480, 640), 2.0)
        p = pc.depth_to_camera_points(depth, K)
        np.testing.assert_allclose(p[240, 420], [100 * 2.0 / 500, 0, 2.0])  # 100 px right -> +x
        np.testing.assert_allclose(p[340, 320], [0, 100 * 2.0 / 500, 2.0])  # 100 px down -> +y

    def test_world_points_and_dropping_empty_pixels(self):
        depth = np.full((480, 640), np.inf)
        depth[240, 320] = 0.5  # one pixel sees something, straight ahead
        X_WC = geo.look_at((0, 0, 1), (0, 0, 0), up=(1, 0, 0))  # 1 m up, looking down
        rgb = np.zeros((480, 640, 3), np.uint8)
        rgb[240, 320] = (255, 0, 0)
        p_W, colors, pix = pc.camera_points_world(dict(depth=depth, K=K, X_WC=X_WC, rgb=rgb))
        np.testing.assert_allclose(p_W, [[0, 0, 0.5]], atol=1e-12)  # 0.5 m below the camera
        np.testing.assert_allclose(colors, [[1, 0, 0]])
        np.testing.assert_array_equal(pix, [[240, 320]])


if __name__ == "__main__":
    unittest.main()
