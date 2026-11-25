"""sensing.icp_pose: recovers a known block pose from points on its faces.  Pure Python (numpy + scipy).

The scene points are RANDOM points on the faces, not the model's grid: points on the model's own grid let ICP lock
one grid step off, which a camera (whose pixels fall anywhere) would not do.
"""

import unittest

import numpy as np

from jenga import geometry as geo
from jenga import sensing
from tests.helpers import make_cfg


def face_points(cfg, faces, n=1500, seed=0):
    """n random points on each named face of a block, in its body frame (origin at the bottom-face centre)."""
    L, W, H = geo.block_dims(cfg)
    rng = np.random.default_rng(seed)
    out = []
    for f in faces:
        u = rng.random((n, 2))
        x, y, z = (u[:, 0] - 0.5) * L, (u[:, 1] - 0.5) * W, u[:, 1] * H
        out.append({"top": np.c_[x, y, np.full(n, H)],
                    "side": np.c_[x, np.full(n, -W / 2), z],
                    "end": np.c_[np.full(n, -L / 2), y, z]}[f])
    return np.vstack(out)


def moved(points, X):
    return points @ X[:3, :3].T + X[:3, 3]


def start_off(X, shift_m, turn_deg):
    """X turned about vertical and shifted sideways and up: a wrong starting guess."""
    X0 = X @ geo.make_pose(0, 0, 0, np.radians(turn_deg))
    X0[:3, 3] += np.array([1.0, 1.0, 0.3]) * shift_m / np.sqrt(2.09)
    return X0


class TestIcpPose(unittest.TestCase):
    def setUp(self):
        self.cfg = make_cfg()
        self.model = sensing.block_model_points(self.cfg)
        self.X_true = geo.make_pose(0.30, 0.10, 0.20, yaw=0.40)

    def error(self, X):
        """(position error in the block's own axes (along, across, up), m; rotation error, deg)"""
        d = self.X_true[:3, :3].T @ (X[:3, 3] - self.X_true[:3, 3])
        c = (np.trace(X[:3, :3].T @ self.X_true[:3, :3]) - 1) / 2
        return d, np.degrees(np.arccos(np.clip(c, -1, 1)))

    def test_returns_a_proper_pose(self):
        X = sensing.icp_pose(moved(face_points(self.cfg, ["top", "side", "end"]), self.X_true), self.model,
                             start_off(self.X_true, 0.003, 2))
        self.assertEqual(X.shape, (4, 4))
        np.testing.assert_allclose(X[3], [0, 0, 0, 1])
        np.testing.assert_allclose(X[:3, :3].T @ X[:3, :3], np.eye(3), atol=1e-9)
        self.assertAlmostEqual(np.linalg.det(X[:3, :3]), 1.0, places=9)

    def test_recovers_the_pose_when_an_end_face_is_seen(self):
        # top, side and end pin the block down in every direction (measured: 0.02-0.03 mm, 0.13-0.16 deg)
        scene = moved(face_points(self.cfg, ["top", "side", "end"]), self.X_true)
        for shift_m, turn_deg in ((0.003, 2), (0.010, 5)):
            d, ang = self.error(sensing.icp_pose(scene, self.model, start_off(self.X_true, shift_m, turn_deg)))
            self.assertLess(np.linalg.norm(d), 0.0002, (shift_m, turn_deg))
            self.assertLess(ang, 0.3, (shift_m, turn_deg))

    def test_without_an_end_face_height_and_across_stay_accurate(self):
        # the known limit: with only the top and a side, ICP settles 1-2 mm off ALONG the block (measured 1.5 mm
        # at 100 iterations), but height (finger margin 0.875 mm) and across stay well under 0.1 mm
        scene = moved(face_points(self.cfg, ["top", "side"]), self.X_true)
        d, ang = self.error(sensing.icp_pose(scene, self.model, start_off(self.X_true, 0.003, 2)))
        self.assertLess(abs(d[1]), 0.0001)
        self.assertLess(abs(d[2]), 0.0001)
        self.assertLess(abs(d[0]), 0.003)
        self.assertLess(ang, 0.3)

    def test_a_perfect_start_stays_put(self):
        scene = moved(face_points(self.cfg, ["top", "side", "end"]), self.X_true)
        d, ang = self.error(sensing.icp_pose(scene, self.model, self.X_true.copy()))
        self.assertLess(np.linalg.norm(d), 0.0002)
        self.assertLess(ang, 0.3)


if __name__ == "__main__":
    unittest.main()
