"""push.plan_push backs out of the slot no faster than it pushed.  Needs Drake; skipped without it."""

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


if __name__ == "__main__":
    unittest.main()
