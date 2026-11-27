"""sensing.block_model_points: the block surface model ICP matches against.  Pure Python."""

import unittest

import numpy as np

from jenga import geometry as geo
from jenga import sensing
from tests.helpers import make_cfg


class TestBlockModelPoints(unittest.TestCase):
    def setUp(self):
        self.cfg = make_cfg()
        self.L, self.W, self.H = geo.block_dims(self.cfg)
        self.P = sensing.block_model_points(self.cfg)

    def test_shape(self):
        self.assertEqual(self.P.ndim, 2)
        self.assertEqual(self.P.shape[1], 3)

    def test_inside_the_block_in_the_body_frame(self):
        # body frame: origin at the centre of the BOTTOM face, so z runs 0..H (not -H/2..H/2)
        x, y, z = self.P.T
        eps = 1e-12
        self.assertTrue(np.all(np.abs(x) <= self.L / 2 + eps))
        self.assertTrue(np.all(np.abs(y) <= self.W / 2 + eps))
        self.assertTrue(np.all((z >= -eps) & (z <= self.H + eps)))

    def test_every_point_is_on_a_face(self):
        x, y, z = self.P.T
        on = (np.isclose(np.abs(x), self.L / 2) | np.isclose(np.abs(y), self.W / 2)
              | np.isclose(z, 0.0) | np.isclose(z, self.H))
        self.assertTrue(on.all())

    def test_every_face_has_points(self):
        x, y, z = self.P.T
        for name, on in (("top", np.isclose(z, self.H)), ("bottom", np.isclose(z, 0.0)),
                         ("+y side", np.isclose(y, self.W / 2)), ("-y side", np.isclose(y, -self.W / 2)),
                         ("+x end", np.isclose(x, self.L / 2)), ("-x end", np.isclose(x, -self.L / 2))):
            self.assertGreater(on.sum(), 0, name)

    def test_spacing_is_at_most_the_one_asked_for(self):
        for spacing in (0.002, 0.005):
            P = sensing.block_model_points(self.cfg, spacing_m=spacing)
            for axis in range(3):
                values = np.unique(P[:, axis].round(9))
                self.assertLessEqual(np.diff(values).max(), spacing + 1e-9, (spacing, axis))


if __name__ == "__main__":
    unittest.main()
