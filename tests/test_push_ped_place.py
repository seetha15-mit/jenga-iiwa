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
        # the plan may end with a still hold at the top, so find the last leg that rises
        zs = [w.X_RG.translation()[2] for w in self.wps]
        lift = max(i for i in range(1, len(zs)) if zs[i] > zs[i - 1] + 1e-9)
        self.assertEqual(zs[-1], max(zs))
        self.assertEqual(self.wps[lift - 1].opening, self.cfg.FINGER_OPEN_COMMAND_M)

    def test_release_is_below_the_post_top(self):
        # the block bottom at the set point sits 2 mm under PEDESTAL_HEIGHT_M (see plan_ped_place)
        hold = push.plan_pull(RigidTransform(), self.cfg)  # same grasp offset X_BG as plan_ped_place
        dz = hold[1].X_RG.translation()[2]                   # hand height above a block whose bottom is at z = 0
        set_z = self.wps[1].X_RG.translation()[2]
        self.assertAlmostEqual(set_z - dz, self.cfg.PEDESTAL_HEIGHT_M - 0.002, places=6)


@unittest.skipUnless(HAVE_DRAKE, "needs pydrake")
class TestPedTarget(unittest.TestCase):
    def test_default_target_is_the_post_2_mm_down(self):
        cfg = make_cfg()
        X = push.pedestal_target(cfg)
        d = cfg.PEDESTAL_DISTANCE_M / np.sqrt(2)
        np.testing.assert_allclose(X.translation(), [cfg.STATION_STANDOFF_M - d, d, cfg.PEDESTAL_HEIGHT_M - 0.002])

    def test_the_plan_moves_rigidly_with_its_target(self):
        cfg = make_cfg()
        base_plan = push.plan_ped_place(RigidTransform(), cfg)
        shift = np.array([0.003, -0.002, 0.012])  # e.g. the block slid 12 mm down the fingers
        X_T = push.pedestal_target(cfg)
        moved = push.plan_ped_place(RigidTransform(), cfg, target=RigidTransform(X_T.rotation(), X_T.translation() + shift))
        for a, b in zip(base_plan, moved):
            np.testing.assert_allclose(b.X_RG.translation() - a.X_RG.translation(), shift, atol=1e-12)
            self.assertEqual(a.opening, b.opening)


if __name__ == "__main__":
    unittest.main()
