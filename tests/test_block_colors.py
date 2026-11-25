"""geometry.block_color: touching blocks differ in "colored", wood keeps two tones by layer.  Pure Python."""

import unittest

from jenga import geometry as geo
from tests.helpers import make_cfg


def touching(a, b):
    """Blocks a, b touch in the starting tower: slot neighbours in a layer, or any pair in adjacent layers."""
    (la, sa), (lb, sb) = divmod(a, 3), divmod(b, 3)
    return (la == lb and abs(sa - sb) == 1) or abs(la - lb) == 1


class TestBlockColors(unittest.TestCase):
    def test_unique_is_the_default(self):
        self.assertEqual(make_cfg().BLOCK_COLORS, "unique")

    def test_unique_every_block_differs(self):
        for n_layers in (3, 6, 12):
            cfg = make_cfg(BLOCK_COLORS="unique", N_LAYERS_START=n_layers)
            self.assertEqual(len({geo.block_color(cfg, b) for b in range(3 * n_layers)}), 3 * n_layers)

    def test_unique_touching_blocks_are_two_hue_steps_apart(self):
        # 18 blocks: hues 20 deg apart; touching blocks get at least 2 steps (40 deg), so an edge pixel blending
        # two touching blocks is not just halfway to the next hue
        self.assertEqual(geo.unique_hue_step(18), 5)
        n, k = 18, geo.unique_hue_step(18)
        for a in range(n):
            for b in range(a + 1, n):
                if touching(a, b):
                    steps = (a - b) * k % n
                    self.assertGreaterEqual(min(steps, n - steps), 2, (a, b))

    def test_unique_hue_step_shares_no_factor(self):
        import math
        for n in (3, 9, 18, 36):
            self.assertEqual(math.gcd(geo.unique_hue_step(n), n), 1, n)

    def test_colored_touching_blocks_differ_for_any_height(self):
        cfg = make_cfg(BLOCK_COLORS="colored")
        for n_layers in (1, 3, 6, 18):
            ids = range(3 * n_layers)
            for a in ids:
                for b in ids:
                    if a < b and touching(a, b):
                        self.assertNotEqual(geo.block_color(cfg, a), geo.block_color(cfg, b), (n_layers, a, b))

    def test_colored_uses_six_colours_by_slot_and_layer_parity(self):
        cfg = make_cfg(BLOCK_COLORS="colored")
        self.assertEqual(len({geo.block_color(cfg, b) for b in range(3 * 12)}), 6)
        self.assertEqual(geo.block_color(cfg, 0), geo.block_color(cfg, 6))  # layer 0 slot 0 = layer 2 slot 0
        self.assertEqual(geo.block_color(cfg, 4), geo.block_color(cfg, 10))  # layer 1 slot 1 = layer 3 slot 1

    def test_wood_is_two_tones_by_layer(self):
        cfg = make_cfg(BLOCK_COLORS="wood")
        self.assertEqual({geo.block_color(cfg, b) for b in range(3)}, {geo.WOOD[0]})
        self.assertEqual({geo.block_color(cfg, b) for b in range(3, 6)}, {geo.WOOD[1]})
        self.assertEqual(geo.block_color(cfg, 6), geo.WOOD[0])

    def test_rgb_values_are_in_range(self):
        for c in geo.COLORED_EVEN + geo.COLORED_ODD + geo.WOOD:
            self.assertEqual(len(c), 3)
            self.assertTrue(all(0.0 <= v <= 1.0 for v in c))


if __name__ == "__main__":
    unittest.main()
