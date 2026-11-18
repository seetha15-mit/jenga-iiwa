import importlib
import unittest
from pathlib import Path

import numpy as np

from jenga import strategies
from jenga.methods import end_pinch, push
from jenga.tower import TowerState
from tests.helpers import make_cfg

STUB_FILES = [
    "jenga/strategies/rule_based.py", "jenga/strategies/perception.py", "jenga/strategies/force_guided.py",
    "jenga/methods/end_pinch.py", "jenga/clearance.py", "jenga/evaluation/collapse.py",
]


class TestStrategies(unittest.TestCase):
    def test_random_picks_only_candidates_and_is_reproducible(self):
        cfg, t = make_cfg(), TowerState(make_cfg())
        select = strategies.get_strategy("random")
        cands = [0, 2, 3]
        a = [select(t, cands, cfg, np.random.default_rng(7)) for _ in range(5)]
        b = [select(t, cands, cfg, np.random.default_rng(7)) for _ in range(5)]
        self.assertEqual(a, b)
        self.assertTrue(all(x in cands for x in a))

    def test_team_stubs_raise_not_implemented(self):
        cfg, t = make_cfg(), TowerState(make_cfg())
        for name in ("perception", "force_guided"):  # rule_based is written (Katie)
            with self.assertRaises(NotImplementedError):
                strategies.get_strategy(name)(t, [0], cfg, np.random.default_rng(0))


class TestRuleBased(unittest.TestCase):
    """Katie's rule-based strategy: the layer-stability rule and the basic contract of select_block."""

    def setUp(self):
        self.cfg = make_cfg(N_LAYERS_START=3)  # block ids below are for a 3-layer tower
        self.t = TowerState(self.cfg)

    def test_layer_stable(self):
        from jenga.strategies import rule_based
        self.assertTrue(rule_based.layer_stable(self.t, 0))  # full layer 0: blocks 1 and 2 stay
        self.t.place(1, 3, 0)  # take the middle of layer 0 away: blocks 0 and 2 (edges) are left
        self.assertFalse(rule_based.layer_stable(self.t, 0))  # only edge 2 would be left
        self.assertFalse(rule_based.layer_stable(self.t, 2))

    def test_never_leaves_a_lone_edge_and_picks_a_candidate(self):
        select = strategies.get_strategy("rule_based")
        self.t.place(1, 3, 0)
        cands = [b for b in self.t.pull_candidates() if end_pinch.can_pull(self.t, b, self.cfg)]  # 0, 2, 3, 5
        picks = {select(self.t, cands, self.cfg, np.random.default_rng(s)) for s in range(20)}
        self.assertTrue(picks <= set(cands))
        self.assertFalse(picks & {0, 2})

    def test_a_block_set_down_end_for_end_has_not_moved(self):
        from jenga.strategies import rule_based
        X = self.t.target_pose(0, 0) @ np.diag([-1.0, -1.0, 1.0, 1.0])  # turned 180 deg about its own z
        self.t.update_poses({0: X})
        self.assertAlmostEqual(rule_based.blockdisturbance(self.t, 0, self.cfg), 0.0, places=6)
        tilt = np.eye(4)
        tilt[:3, :3] = [[1, 0, 0], [0, np.cos(0.0349), -np.sin(0.0349)], [0, np.sin(0.0349), np.cos(0.0349)]]
        self.t.update_poses({0: self.t.target_pose(0, 0) @ tilt})  # a real 2 deg tilt still counts
        self.assertAlmostEqual(rule_based.blockdisturbance(self.t, 0, self.cfg), 1.0, places=2)

    def test_falls_back_when_no_pull_is_safe(self):
        select = strategies.get_strategy("rule_based")
        self.t.place(1, 3, 0)  # layer 0: only the edges 0 and 2 left, neither is safe to pull
        self.assertIn(select(self.t, [0, 2], self.cfg, np.random.default_rng(0)), (0, 2))

    def test_reproducible_with_the_same_seed(self):
        select = strategies.get_strategy("rule_based")
        cands = [0, 2, 3, 5]
        a = [select(self.t, cands, self.cfg, np.random.default_rng(3)) for _ in range(5)]
        b = [select(self.t, cands, self.cfg, np.random.default_rng(3)) for _ in range(5)]
        self.assertEqual(a, b)

    def test_push_places_like_end_pinch(self):
        # after the regrab the block is held end-pinch, so push uses end-pinch's place rules (D-21, D-22)
        cfg, t = make_cfg(), TowerState(make_cfg())
        for layer, slot in t.placement_slots():
            self.assertEqual(push.can_place(t, layer, slot, cfg), end_pinch.can_place(t, layer, slot, cfg))
            for k in range(len(cfg.STATION_ANGLES_DEG)):
                self.assertEqual(push.choose_place_station(t, layer, slot, cfg, k),
                                 end_pinch.choose_place_station(t, layer, slot, cfg, k))

    def test_push_has_all_five_steps(self):
        # RealMover runs these in order for a push turn (D-21); the waypoints themselves are checked in Drake
        for name in ("plan_push", "plan_pull", "plan_ped_place", "plan_regrab", "plan_place"):
            self.assertTrue(callable(getattr(push, name, None)), name)

    def test_unknown_strategy(self):
        with self.assertRaises(ValueError):
            strategies.get_strategy("nope")


class TestRepoHygiene(unittest.TestCase):
    def test_every_stub_file_is_marked_for_the_team(self):
        for f in STUB_FILES:
            text = Path(f).read_text()
            self.assertIn("BEGIN CODE TO BE WRITTEN", text, f)
            self.assertIn("END CODE TO BE WRITTEN", text, f)

    def test_every_python_file_has_a_docstring(self):
        for p in list(Path("jenga").rglob("*.py")) + [Path("config.py"), Path("run_checks.py"), Path("run_experiment.py")]:
            text = p.read_text().lstrip()
            if p.name == "__init__.py" and not text:
                continue
            self.assertTrue(text.startswith('"""'), f"{p} has no module docstring")

    def test_pure_modules_import_without_drake(self):
        for mod in ("jenga.geometry", "jenga.tower", "jenga.base", "jenga.methods.end_pinch", "jenga.mover",
                    "jenga.evaluation.harness", "jenga.evaluation.results", "jenga.drake_checks", "jenga.config_check"):
            importlib.import_module(mod)

    def test_docs_exist_and_decisions_is_short(self):
        for name in ("SETUP.md", "HOW_IT_WORKS.md", "FOR_THE_TEAM.md", "DECISIONS.md"):
            self.assertTrue(Path("docs", name).exists(), name)
        self.assertLessEqual(len(Path("docs/DECISIONS.md").read_text().splitlines()), 110)  # about two pages


if __name__ == "__main__":
    unittest.main()
