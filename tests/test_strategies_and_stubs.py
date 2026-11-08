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
        for name in ("rule_based", "perception", "force_guided"):
            with self.assertRaises(NotImplementedError):
                strategies.get_strategy(name)(t, [0], cfg, np.random.default_rng(0))

    def test_push_is_a_placeholder(self):
        # only the station rules are written (can_pull, choose_station, choose_pull_station); the rest are stubs
        stubs = ((push.can_place, (None, 0, 0, None)), (push.choose_place_station, (None, 0, 0, None, 0)),
                 (push.plan_push, (None, None)), (push.plan_pull, (None, None)),
                 (push.plan_regrab, (None, None)), (push.plan_place, (None, None)))
        for fn, args in stubs:
            with self.assertRaises(NotImplementedError):
                fn(*args)

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
