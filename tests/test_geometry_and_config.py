import tempfile
import unittest
from pathlib import Path

import numpy as np

from jenga import config_check, geometry as geo
from tests.helpers import make_cfg


class TestGeometry(unittest.TestCase):
    def test_block_dims_one_scale(self):
        cfg = make_cfg()
        L, W, H = geo.block_dims(cfg)
        self.assertAlmostEqual(L / 0.075, W / 0.025)
        self.assertAlmostEqual(L / 0.075, H / 0.015)
        self.assertAlmostEqual(L, 0.10875, places=6)

    def test_margins_match_decision(self):
        cfg = make_cfg()  # option B: s = 1.45, command 110 mm
        self.assertAlmostEqual(geo.pad_gap_m(cfg), 0.115)
        self.assertAlmostEqual(geo.jaw_margin_per_side_m(cfg) * 1000, 3.125, places=3)
        self.assertAlmostEqual(geo.finger_height_margin_per_side_m(cfg) * 1000, 0.875, places=3)

    def test_mass_is_light_enough(self):
        self.assertLess(geo.block_mass(make_cfg()), 0.1)

    def test_far_slot_hits_the_hand_body_near_and_middle_do_not(self):
        cfg = make_cfg()
        _, W, _ = geo.block_dims(cfg)
        step = W + cfg.BLOCK_GAP_M
        self.assertFalse(geo.housing_collides(cfg, -step))  # near edge slot
        self.assertFalse(geo.housing_collides(cfg, 0.0))  # middle slot
        self.assertTrue(geo.housing_collides(cfg, +step))  # far edge slot
        overlap, dip = geo.housing_overlap(cfg, +step)
        self.assertGreater(overlap, 0.015)  # about 20 mm (derived)
        self.assertAlmostEqual(dip * 1000, 14.1, delta=0.2)

    def test_friction_is_reproducible_and_hidden_modes(self):
        cfg = make_cfg(FRICTION_MODE="random")
        self.assertEqual(geo.assign_friction(cfg, 5), geo.assign_friction(cfg, 5))
        self.assertNotEqual(geo.assign_friction(cfg, 5), geo.assign_friction(cfg, 6))
        lo, hi = cfg.FRICTION_RANGE
        self.assertTrue(all(lo <= v <= hi for v in geo.assign_friction(cfg, 5).values()))
        self.assertEqual(set(geo.assign_friction(make_cfg(), 0).values()), {0.5})


class TestConfig(unittest.TestCase):
    def test_repo_config_has_no_untagged_settings(self):
        self.assertEqual(config_check.check_tags(Path("config.py")), [])

    def test_untagged_setting_is_flagged(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "c.py"
            p.write_text("GOOD = 1  # [user] fine\nBAD = 2  # no tag here\nlower = 3\n")
            w = config_check.check_tags(p)
            self.assertEqual(len(w), 1)
            self.assertIn("BAD", w[0])

    def test_repo_config_values_only_warn_about_the_fake_mover(self):
        w = config_check.check_values(make_cfg())
        self.assertEqual([x for x in w if "MOVER = 'fake'" not in x], [])

    def test_bad_values_are_flagged(self):
        self.assertTrue(any("does not fit" in x for x in config_check.check_values(make_cfg(BLOCK_SCALE=1.6))))
        self.assertTrue(any("lower than the fingers" in x for x in config_check.check_values(make_cfg(BLOCK_SCALE=1.2))))
        self.assertTrue(any("must be one of" in x for x in config_check.check_values(make_cfg(STRATEGY="nope"))))
        self.assertTrue(any("roll through the tower" in x for x in config_check.check_values(make_cfg(BASE_TRAVEL_RADIUS_M=0.3))))

    def test_describe_runs(self):
        self.assertIn("jaw", config_check.describe(make_cfg()))


if __name__ == "__main__":
    unittest.main()
