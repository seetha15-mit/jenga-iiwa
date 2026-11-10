"""push.plan_ped_place releases the block with the hand still.  Needs Drake; skipped without it."""

import unittest

import numpy as np

try:
    from pydrake.all import RigidTransform
    from jenga.methods import push
    HAVE_DRAKE = True
except ImportError:
    HAVE_DRAKE = False

from tests.helpers import make_cfg


@unittest.skipUnless(HAVE_DRAKE, "needs pydrake")
class TestPedPlace(unittest.TestCase):
    def setUp(self):
        self.cfg = make_cfg()
        self.wps = push.plan_ped_place(RigidTransform(), self.cfg)

    def test_fingers_open_only_on_still_legs(self):
        # an opening is commanded during the NEXT leg (jenga/waypoint.py); opening while lifting flicked the
        # block off the post, so every leg that follows a wider-than-before opening must not move the hand
        for i in range(1, len(self.wps) - 1):
            prev, cur, nxt = self.wps[i - 1], self.wps[i], self.wps[i + 1]
            if cur.opening > prev.opening:
                moved = np.linalg.norm(nxt.X_RG.translation() - cur.X_RG.translation())
                self.assertLess(moved, 1e-9, f"leg after waypoint {i} moves while the fingers open")

    def test_squeeze_is_released_before_opening(self):
        # the first opening after the grip leaves the pads clear of the block (pad gap = opening + 5 mm)
        _, W, _ = push.geo.block_dims(self.cfg)
        self.assertGreaterEqual(self.wps[2].opening + 0.005, W)
        self.assertLess(self.wps[2].opening, self.cfg.FINGER_OPEN_COMMAND_M)

    def test_lift_comes_last_and_hand_is_open(self):
        last, before = self.wps[-1], self.wps[-2]
        self.assertGreater(last.X_RG.translation()[2], before.X_RG.translation()[2])
        self.assertEqual(before.opening, self.cfg.FINGER_OPEN_COMMAND_M)

    def test_release_is_below_the_post_top(self):
        # the block bottom at the set point sits 2 mm under PEDESTAL_HEIGHT_M (see plan_ped_place)
        hold = push.plan_pull(RigidTransform(), self.cfg)  # same grasp offset X_BG as plan_ped_place
        dz = hold[1].X_RG.translation()[2]                   # hand height above a block whose bottom is at z = 0
        set_z = self.wps[1].X_RG.translation()[2]
        self.assertAlmostEqual(set_z - dz, self.cfg.PEDESTAL_HEIGHT_M - 0.002, places=6)


if __name__ == "__main__":
    unittest.main()
