"""The final run's analysis (cluster/analyze_final.py): parsing, the validity check and the statistics."""
import tempfile
import unittest
from pathlib import Path

import numpy as np

from cluster import analyze_final as af
from cluster import final_conditions as fc


class TestParsing(unittest.TestCase):
    def test_overrides_round_trip_and_mismatch(self):
        text = "DIAG_LOG_LEVEL = 1\nFRICTION_MODE = 'random'\nPROBE_ENABLED = True\n"
        found = af.parse_overrides(text)
        self.assertEqual(found, {"DIAG_LOG_LEVEL": 1, "FRICTION_MODE": "random", "PROBE_ENABLED": True})
        self.assertEqual(af.check_overrides(found, fc.overrides("E")), [])
        self.assertEqual(af.check_overrides(found, fc.overrides("D")), ["PROBE_ENABLED = True not in the condition"])
        self.assertIn("FRICTION_MODE missing", af.check_overrides({"DIAG_LOG_LEVEL": 1}, fc.overrides("D"))[0])
        self.assertEqual(af.parse_overrides("none\n"), {})

    def test_every_comparison_names_known_conditions(self):
        for cid, a, b, kind in fc.COMPARISONS:
            self.assertIn(a, fc.CONDITIONS)
            self.assertIn(b, fc.CONDITIONS)
            self.assertIn(kind, ("difference", "equivalence", "secondary"))

    def test_validity_stops_on_mismatch_and_reports_missing(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for letter, commit in (("D", "abc"), ("E", "abd")):
                (root / letter).mkdir()
                (root / letter / "code.txt").write_text(f"{commit} msg\n")
            data = {"D": {0: dict(overrides=fc.overrides("D"))},
                    "E": {0: dict(overrides=fc.overrides("D")), 1: dict(overrides=fc.overrides("E"))}}
            problems, notes = af.validity(root, data, range(0, 2))
            self.assertTrue(any("E trial 0: PROBE_ENABLED missing" in p for p in problems))
            self.assertTrue(any("different commits" in p for p in problems))
            self.assertEqual(notes, ["D: 1 game(s) missing: 1"])


class TestStatistics(unittest.TestCase):
    def test_holm_order_and_cap(self):
        adj = af.holm([0.01, 0.04, 0.03])  # sorted 0.01, 0.03, 0.04 -> 0.03, 0.06, max(0.06, 0.04)
        self.assertEqual([round(x, 6) for x in adj], [0.03, 0.06, 0.06])
        self.assertEqual(af.holm([0.6, 0.5]), [1.0, 1.0])

    def test_wilcoxon_all_zero_is_one(self):
        self.assertEqual(af.wilcoxon_p([0, 0, 0]), 1.0)
        self.assertLess(af.wilcoxon_p([1] * 20 + [2] * 5), 0.001)

    def test_tost_inside_and_outside_the_margin(self):
        rng = np.random.default_rng(0)
        e = af.tost(rng.normal(0.0, 1.0, 400), 0.5)
        self.assertTrue(e["equivalent"])
        self.assertTrue(-0.5 < e["ci90"][0] < e["ci90"][1] < 0.5)
        self.assertFalse(af.tost(rng.normal(0.6, 1.0, 400), 0.5)["equivalent"])

    def test_mean_ci_and_counts(self):
        m, lo, hi = af.mean_ci([1.0, 2.0, 3.0])
        self.assertAlmostEqual(m, 2.0)
        self.assertLess(lo, 2.0)
        self.assertGreater(hi, 2.0)
        self.assertEqual(af.counts([1, -2, 0, 0, 3]), (2, 1, 2))

    def test_paired_uses_trials_in_both(self):
        a = {0: dict(blocks=3), 1: dict(blocks=4)}
        b = {1: dict(blocks=6), 2: dict(blocks=1)}
        both, d, left = af.paired(a, b)
        self.assertEqual((both, d.tolist(), left), ([1], [2.0], 2))

    def test_end_step_prefers_the_error(self):
        steps = [dict(turn="3", waypoint="", step="pull", error="", worst_mm="50"),
                 dict(turn="3", waypoint="2", step="pull", error="", worst_mm="999"),
                 dict(turn="3", waypoint="", step="regrab", error="dropped", worst_mm="1"),
                 dict(turn="2", waypoint="", step="place", error="x", worst_mm="1")]
        self.assertEqual(af.end_step(steps, "3"), "regrab")
        self.assertEqual(af.end_step([s for s in steps if s["step"] != "regrab"], "3"), "pull")
        self.assertEqual(af.end_step(steps, "9"), "")

    def test_parse_trials(self):
        self.assertEqual(list(af.parse_trials("0-3")), [0, 1, 2, 3])
        self.assertEqual(list(af.parse_trials("7")), [7])


if __name__ == "__main__":
    unittest.main()
