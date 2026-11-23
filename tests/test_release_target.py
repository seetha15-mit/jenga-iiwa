"""mover.release_target aims a place at the measured top of the layer below, not the built height (D-25)."""

import unittest

import numpy as np

from jenga import mover
from jenga.tower import TowerState
from tests.helpers import make_cfg


class TestReleaseTarget(unittest.TestCase):
    def setUp(self):
        self.cfg = make_cfg(N_LAYERS_START=3)
        self.t = TowerState(self.cfg)
        self.H = self.t.dims[2]
        self.poses = {b: self.t.target_pose(r.layer, r.slot) for b, r in self.t.blocks.items()}

    def test_aims_one_block_height_above_the_measured_layer_below(self):
        for b in self.t.layer_blocks(2):  # the top layer has settled 1.0 mm
            self.poses[b] = self.poses[b].copy()
            self.poses[b][2, 3] -= 0.001
        X = mover.release_target(self.t, 3, 0, self.poses, self.cfg)
        built = self.t.target_pose(3, 0)
        self.assertAlmostEqual(X[2, 3], self.t.target_pose(2, 0)[2, 3] - 0.001 + self.H)
        self.assertLess(X[2, 3], built[2, 3])  # below the built height, which still has the gap
        np.testing.assert_allclose(X[:2, :], built[:2, :])  # only the height changes

    def test_the_held_block_is_not_counted(self):
        held = self.t.layer_blocks(2)[0]
        self.poses[held] = self.poses[held].copy()
        self.poses[held][2, 3] += 0.5  # up in the air, in the hand
        X = mover.release_target(self.t, 3, 0, self.poses, self.cfg, held=held)
        self.assertAlmostEqual(X[2, 3], self.t.target_pose(2, 0)[2, 3] + self.H)


if __name__ == "__main__":
    unittest.main()
