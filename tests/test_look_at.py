"""geometry.look_at: camera poses for the RGB-D cameras.  Pure Python."""

import unittest

import numpy as np

from jenga import geometry as geo


class TestLookAt(unittest.TestCase):
    def test_level_camera_looking_along_x(self):
        X = geo.look_at((0, 0, 1), (2, 0, 1))
        np.testing.assert_allclose(X[:3, 2], [1, 0, 0], atol=1e-12)   # view along +x
        np.testing.assert_allclose(X[:3, 1], [0, 0, -1], atol=1e-12)  # image down = world down
        np.testing.assert_allclose(X[:3, 0], [0, -1, 0], atol=1e-12)  # image right = world -y (right, facing +x)
        np.testing.assert_allclose(X[:3, 3], [0, 0, 1])

    def test_rotation_is_proper_and_points_at_target(self):
        eye, target = np.array([0.3, 0.35, 0.55]), np.array([0.95, 0.0, 0.2])
        X = geo.look_at(eye, target)
        R = X[:3, :3]
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(R), 1.0)
        np.testing.assert_allclose(R[:, 2], (target - eye) / np.linalg.norm(target - eye))
        self.assertLess(R[2, 1], 0)  # image down still points down in the world

    def test_straight_down_needs_a_horizontal_up(self):
        X = geo.look_at((0, 0, 1), (0, 0, 0), up=(1, 0, 0))
        np.testing.assert_allclose(X[:3, 2], [0, 0, -1], atol=1e-12)
        np.testing.assert_allclose(X[:3, 1], [-1, 0, 0], atol=1e-12)  # image top = world +x
        with self.assertRaises(ValueError):
            geo.look_at((0, 0, 1), (0, 0, 0))


if __name__ == "__main__":
    unittest.main()
