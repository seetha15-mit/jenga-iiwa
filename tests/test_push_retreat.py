"""push.plan_push backs out of the slot no faster than it pushed; push.plan_pull's pull-out speeds.  Needs Drake;
skipped without it."""

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
class TestPushRetreat(unittest.TestCase):
    def test_retreat_from_the_slot_is_as_slow_as_the_push(self):
        cfg = make_cfg()
        L = push.geo.block_dims(cfg)[0]
        wps = push.plan_push(RigidTransform([0.9, 0.0, 0.3]), cfg)
        start, pushed, out = (w.X_RG.translation() for w in wps[1:4])
        push_speed = np.linalg.norm(pushed - start) / wps[2].seconds
        retreat_speed = np.linalg.norm(out - pushed) / wps[3].seconds
        self.assertAlmostEqual(np.linalg.norm(pushed - start), 0.4 * L)
        self.assertLessEqual(retreat_speed, push_speed + 1e-9)
        # it ends with the fingertips outside the tower: behind the push's start, on the same line
        self.assertGreater(np.linalg.norm(out - pushed), np.linalg.norm(start - pushed))
        self.assertAlmostEqual(np.linalg.norm(np.cross(out - pushed, start - pushed)), 0.0, places=9)


@unittest.skipUnless(HAVE_DRAKE, "needs pydrake")
class TestPullOut(unittest.TestCase):
    def _legs(self, cfg):
        wps = push.plan_pull(RigidTransform([0.9, 0.0, 0.3]), cfg)
        lift, inside, out = (w.X_RG.translation() for w in wps[3:6])
        return lift, inside, out, wps[4].seconds, wps[5].seconds

    def test_at_0_1_m_s_it_is_the_old_pull_200_mm_in_2_s_on_one_line(self):
        lift, inside, out, t_in, t_out = self._legs(make_cfg(PULL_INSIDE_SPEED_MS=0.1, PULL_OUT_SPEED_MS=0.1))
        self.assertAlmostEqual(np.linalg.norm(out - lift), 0.2)
        self.assertAlmostEqual(t_in + t_out, 0.2 / 0.1)
        self.assertAlmostEqual(np.linalg.norm(np.cross(inside - lift, out - lift)), 0.0, places=9)

    def test_default_pulls_slowly_while_the_block_is_inside(self):
        cfg = make_cfg()
        lift, inside, _, t_in, _ = self._legs(cfg)
        self.assertAlmostEqual(np.linalg.norm(inside - lift) / t_in, cfg.PULL_INSIDE_SPEED_MS)
        self.assertLess(cfg.PULL_INSIDE_SPEED_MS, cfg.PULL_OUT_SPEED_MS)

    def test_inside_part_ends_with_the_block_clear_and_has_its_own_speed(self):
        cfg = make_cfg(PULL_INSIDE_SPEED_MS=0.02, PULL_OUT_SPEED_MS=0.4)
        L = push.geo.block_dims(cfg)[0]
        lift, inside, out, t_in, t_out = self._legs(cfg)
        self.assertAlmostEqual(np.linalg.norm(inside - lift), 0.6 * L + 0.010)
        self.assertAlmostEqual(np.linalg.norm(inside - lift) / t_in, 0.02)
        self.assertAlmostEqual(np.linalg.norm(out - inside) / t_out, 0.4)


if __name__ == "__main__":
    unittest.main()
