"""sensing.segment_blocks: splits camera points into blocks by hue.  Pure Python (numpy + matplotlib).

Synthetic points with known colours stand in for a camera; the real cameras are checked by
scratch/check_perception.py (fresh tower: 16 of 18 blocks 93-99.9% their own points, blocks 1 and 13 76-78% from
blended top-layer edges, which estimate_block_poses removes with each block's guessed box).
"""

import unittest

import numpy as np

from jenga import geometry as geo
from jenga import sensing
from tests.helpers import make_cfg


class TestSegmentBlocks(unittest.TestCase):
    def setUp(self):
        self.cfg = make_cfg(BLOCK_COLORS="unique")
        self.n = 3 * self.cfg.N_LAYERS_START
        cx, cy = self.cfg.TOWER_CENTER_XY
        self.z = self.cfg.PLATFORM_HEIGHT_M + 0.05  # inside the tower box
        rng = np.random.default_rng(0)
        pts, rgb, truth = [], [], []
        for b in range(self.n):  # 20 points per block, each at its own spot so it can be traced
            p = np.c_[cx + rng.uniform(-0.03, 0.03, 20), cy + rng.uniform(-0.03, 0.03, 20), np.full(20, self.z)]
            pts.append(p)
            rgb.append(np.tile(geo.block_color(self.cfg, b), (20, 1)))
            truth += [b] * 20
        self.pts, self.rgb, self.truth = np.vstack(pts), np.vstack(rgb), np.array(truth)

    def test_every_block_gets_exactly_its_own_points(self):
        seg = sensing.segment_blocks(self.pts, self.rgb, self.cfg)
        self.assertEqual(set(seg), set(range(self.n)))
        for b in range(self.n):
            np.testing.assert_allclose(np.sort(seg[b], axis=0), np.sort(self.pts[self.truth == b], axis=0))

    def test_shaded_colours_keep_their_block(self):
        # shading darkens a face (side faces render at ~40% of the top's brightness): the hue, and so the block, stays
        seg = sensing.segment_blocks(self.pts, self.rgb * 0.4, self.cfg)
        for b in range(self.n):
            self.assertEqual(len(seg[b]), 20, b)

    def test_grey_points_are_dropped(self):
        # table colour (saturation ~0.18), inside the tower box
        grey = np.tile([0.55, 0.50, 0.45], (50, 1))
        seg = sensing.segment_blocks(np.vstack([self.pts, self.pts[:50]]), np.vstack([self.rgb, grey]), self.cfg)
        self.assertEqual(sum(len(v) for v in seg.values()), len(self.pts))

    def test_coloured_points_outside_the_tower_box_are_dropped(self):
        # e.g. the orange arm: strongly coloured, but well away from the tower
        far = self.pts[:50] + [0.5, 0.0, 0.0]
        orange = np.tile([0.9, 0.45, 0.1], (50, 1))
        seg = sensing.segment_blocks(np.vstack([self.pts, far]), np.vstack([self.rgb, orange]), self.cfg)
        self.assertEqual(sum(len(v) for v in seg.values()), len(self.pts))

    def test_points_below_the_platform_top_are_dropped(self):
        low = self.pts.copy()
        low[:, 2] = self.cfg.PLATFORM_HEIGHT_M - 0.01
        seg = sensing.segment_blocks(low, self.rgb, self.cfg)
        self.assertEqual(sum(len(v) for v in seg.values()), 0)


if __name__ == "__main__":
    unittest.main()
