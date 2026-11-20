"""The place target is raised by PLACE_RELEASE_LIFT_M, and the grasp correction keeps that lift."""

import unittest

import numpy as np

from jenga import mover
from jenga.tower import TowerState
from tests.helpers import make_cfg


class TestReleaseLift(unittest.TestCase):
    def test_release_target_is_the_slot_raised_by_the_lift_only(self):
        cfg = make_cfg(PLACE_RELEASE_LIFT_M=0.001)
        t = TowerState(cfg)
        layer, slot = t.placement_slots()[0]
        d = mover.release_target(t, layer, slot, cfg) - t.target_pose(layer, slot)
        expected = np.zeros((4, 4))
        expected[2, 3] = 0.001
        np.testing.assert_allclose(d, expected, atol=1e-12)

    def test_grasp_correction_keeps_the_lift(self):
        # no slip in the hand: the corrected target is the lifted target itself, not the slot
        cfg = make_cfg(PLACE_RELEASE_LIFT_M=0.001)
        t = TowerState(cfg)
        X_RT = mover.release_target(t, *t.placement_slots()[0], cfg)
        X_RG = X_RT @ np.array([[1, 0, 0, 0], [0, -1, 0, 0.05], [0, 0, -1, 0.01], [0, 0, 0, 1.0]])
        np.testing.assert_allclose(mover.grasp_corrected_target(X_RT, X_RG, X_RG, X_RT), X_RT, atol=1e-12)


if __name__ == "__main__":
    unittest.main()
