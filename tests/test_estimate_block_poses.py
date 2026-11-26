"""sensing.estimate_block_poses: cameras -> segment -> per-block box -> ICP.  Pure Python (no Drake, no renderer).

The cameras are faked: pointcloud.camera_points_world is replaced for the test so each "image" hands back points
and colours chosen here (points on a block's faces, in its colour).  The real cameras are checked by
scratch/check_perception.py (fresh tower, ICP started from the settled height: every block within 0.31 mm in height).
"""

import unittest
from unittest import mock

import numpy as np

from jenga import geometry as geo
from jenga import pointcloud, sensing
from tests.helpers import make_cfg


def face_points(cfg, n=800, seed=0):
    """Random points on a block's top, one side and one end, in its body frame."""
    L, W, H = geo.block_dims(cfg)
    rng = np.random.default_rng(seed)
    u, v, w = rng.random((n, 2)), rng.random((n, 2)), rng.random((n, 2))
    return np.vstack([np.c_[(u[:, 0] - .5) * L, (u[:, 1] - .5) * W, np.full(n, H)],
                      np.c_[(v[:, 0] - .5) * L, np.full(n, -W / 2), v[:, 1] * H],
                      np.c_[np.full(n, -L / 2), (w[:, 0] - .5) * W, w[:, 1] * H]])


def moved(points, X):
    return points @ X[:3, :3].T + X[:3, 3]


def fake_cameras(parts):
    """parts: list of (points (N, 3), rgb (N, 3)) -> an `images` dict, and a stand-in for camera_points_world."""
    images = {f"cam{i}": {"points": p, "rgb": c} for i, (p, c) in enumerate(parts)}

    def camera_points_world(image, max_range_m=3.0):
        return image["points"], image["rgb"], np.zeros((len(image["points"]), 2), int)

    return images, camera_points_world


class TestEstimateBlockPoses(unittest.TestCase):
    def setUp(self):
        self.cfg = make_cfg(BLOCK_COLORS="unique")
        L, W, H = geo.block_dims(self.cfg)
        z0 = self.cfg.PLATFORM_HEIGHT_M
        # block 3 (layer 1) and block 6 (layer 2), at the tower centre, crossing each other
        self.truth = {3: geo.make_pose(0.001, -0.002, z0 + H, yaw=np.pi / 2 + 0.01),
                      6: geo.make_pose(-0.001, 0.0015, z0 + 2 * H, yaw=0.005)}
        self.colour = {b: np.array(geo.block_color(self.cfg, b)) for b in self.truth}

    def guesses(self, shift=0.002, turn_deg=1.0):
        out = {}
        for b, X in self.truth.items():
            G = X @ geo.make_pose(0, 0, 0, np.radians(turn_deg))
            G[:3, 3] += [shift, -shift, 0.0005]
            out[b] = G
        return out

    def block_part(self, b, seed=0):
        p = moved(face_points(self.cfg, seed=seed), self.truth[b])
        return p, np.tile(self.colour[b], (len(p), 1))

    def estimate(self, parts, guesses):
        images, fake = fake_cameras(parts)
        with mock.patch.object(pointcloud, "camera_points_world", fake):
            return sensing.estimate_block_poses(images, self.cfg, guesses)

    def assert_close(self, X, b, tol_m=0.0003, tol_deg=0.3):
        d = np.linalg.norm(X[:3, 3] - self.truth[b][:3, 3])
        c = (np.trace(X[:3, :3].T @ self.truth[b][:3, :3]) - 1) / 2
        self.assertLess(d, tol_m, b)
        self.assertLess(np.degrees(np.arccos(np.clip(c, -1, 1))), tol_deg, b)

    def test_finds_each_block_from_a_wrong_start(self):
        est = self.estimate([self.block_part(3), self.block_part(6, seed=1)], self.guesses())
        self.assertEqual(set(est), {3, 6})
        for b in est:
            self.assertEqual(est[b].shape, (4, 4))
            self.assert_close(est[b], b)

    def test_points_from_several_cameras_are_combined(self):
        # block 3's points split across two cameras give the same answer as one camera
        p, c = self.block_part(3)
        est = self.estimate([(p[::2], c[::2]), (p[1::2], c[1::2])], {3: self.guesses()[3]})
        self.assert_close(est[3], 3)

    def test_points_in_a_blocks_colour_far_from_it_are_ignored(self):
        # the blended-edge leak: points in block 3's colour two layers up must not pull its estimate
        p, c = self.block_part(3)
        leak = p[:300] + [0.0, 0.0, 0.05]
        est = self.estimate([(np.vstack([p, leak]), np.vstack([c, c[:300]]))], {3: self.guesses()[3]})
        self.assert_close(est[3], 3)

    def test_a_block_with_too_few_points_is_left_out(self):
        p, c = self.block_part(3)
        est = self.estimate([(p[:20], c[:20]), self.block_part(6, seed=1)], self.guesses())
        self.assertNotIn(3, est)
        self.assertIn(6, est)

    def test_a_block_nobody_sees_is_left_out(self):
        est = self.estimate([self.block_part(6, seed=1)], self.guesses())
        self.assertEqual(set(est), {6})


if __name__ == "__main__":
    unittest.main()
