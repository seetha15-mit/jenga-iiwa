import csv
import tempfile
import unittest
from pathlib import Path

import numpy as np

from jenga import strategies
from jenga.evaluation import collapse, harness, outcomes, results
from jenga.mover import FakeMover
from jenga.tower import TowerState
from tests import helpers
from tests.helpers import MockWorld, make_cfg


def read(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


class TestOutcomesAndCollapse(unittest.TestCase):
    def setUp(self):
        self.cfg = make_cfg()
        self.t = TowerState(self.cfg)

    def test_classify_turn(self):
        c = self.cfg
        before = self.t.target_pose(0, 0)
        target = self.t.target_pose(3, 0)
        self.assertEqual(outcomes.classify_turn(before, target.copy(), target, c), "replaced")
        self.assertEqual(outcomes.classify_turn(before, before.copy(), target, c), "failed_pull")
        mid = before.copy()
        mid[0, 3] += 0.2
        self.assertEqual(outcomes.classify_turn(before, mid, target, c), "dropped")

    def _truth(self, shove=0.0, moved=0):
        before = {b: r.X_WB.copy() for b, r in self.t.blocks.items()}
        after = {b: X.copy() for b, X in before.items()}
        after[4][0, 3] += shove
        after[moved][0, 3] += 0.3  # the moved block is allowed to move a lot
        return collapse.Truth(before, after, self.t, moved)

    def test_basic_detector(self):
        self.assertFalse(collapse.basic(self._truth(0.005), self.cfg).stop)
        v = collapse.basic(self._truth(0.03), self.cfg)
        self.assertTrue(v.stop)
        self.assertIn("block 4", v.reason)

    def test_loose_blocks_are_ignored(self):
        self.t.mark_loose(4)
        self.assertFalse(collapse.basic(self._truth(0.5), self.cfg).stop)

    def test_advanced_is_a_stub_and_registry_works(self):
        with self.assertRaises(NotImplementedError):
            collapse.advanced(self._truth(), self.cfg)
        with self.assertRaises(ValueError):
            collapse.get_detector("nope")


class TestHarness(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        strategies.REGISTRY["lowest"] = "tests.helpers_lowest"
        self.cfg = make_cfg(RECORD_REPLAY=False, RUNS_DIR=str(self.root / "runs"), FAKE_RUNS_DIR=str(self.root / "runs_fake"), MAX_TURNS=6)

    def tearDown(self):
        strategies.REGISTRY.pop("lowest", None)
        self.tmp.cleanup()

    def run_trial(self, mover, trial=0, strategy="random", cfg=None):
        cfg = cfg or self.cfg
        out = harness.run_directory(cfg, mover, strategy, stamp="t")
        s = harness.run_trial(cfg, trial, mover, out, strategy=strategy, make_world=MockWorld, log=lambda *_: None)
        return s, out

    def test_fake_trial_is_tagged_and_goes_to_the_fake_folder(self):
        s, out = self.run_trial(FakeMover())
        self.assertEqual(out.parent.name, "runs_fake")
        self.assertEqual(s["mover"], "fake")
        rows = read(out / "turns.csv")
        self.assertTrue(rows and all(r["mover"] == "fake" for r in rows))
        self.assertTrue(all(r["arm_collisions_unchecked"] == "True" for r in rows))
        self.assertEqual(s["blocks_replaced"], sum(r["outcome"] == "replaced" for r in rows))
        self.assertGreaterEqual(s["blocks_replaced"], 4)  # the course minimum is reachable with the placeholder

    def test_the_tower_stands_before_anything_moves(self):
        worlds = []

        def factory(*args):
            worlds.append(MockWorld(*args))
            return worlds[-1]

        out = harness.run_directory(self.cfg, FakeMover(), "random", stamp="settle")
        harness.run_trial(self.cfg, 0, FakeMover(), out, make_world=factory, log=lambda *_: None)
        events = worlds[0].events
        self.assertEqual(events[0], ("hold", self.cfg.START_SETTLE_S))
        first_teleport = next(e for e in events if e[0] == "teleport")
        self.assertGreaterEqual(first_teleport[2], self.cfg.START_SETTLE_S)  # not at time zero (the replay bug)

    def test_real_mover_goes_to_the_runs_folder(self):
        _, out = self.run_trial(helpers.RealLikeMover())
        self.assertEqual(out.parent.name, "runs")

    def test_seed_fixes_the_first_block_for_every_strategy(self):
        a, _ = self.run_trial(FakeMover(), trial=3, strategy="random")
        b, _ = self.run_trial(FakeMover(), trial=3, strategy="lowest")
        c, _ = self.run_trial(FakeMover(), trial=3, strategy="random")
        self.assertEqual(a["first_block"], b["first_block"])
        self.assertEqual(a["first_block"], c["first_block"])
        firsts = {self.run_trial(FakeMover(), trial=i)[0]["first_block"] for i in range(12)}
        self.assertGreater(len(firsts), 1)  # different seeds do pick different first blocks

    def test_collapse_ends_the_trial_and_does_not_count(self):
        s, out = self.run_trial(helpers.DisturbMover())
        self.assertTrue(s["collapsed"])
        self.assertEqual(s["end_reason"], "collapse")
        self.assertEqual(s["blocks_replaced"], 0)
        self.assertEqual(read(out / "turns.csv")[-1]["outcome"], "collapse")

    def test_consecutive_failures_end_the_trial(self):
        s, out = self.run_trial(helpers.DoNothingMover())
        self.assertEqual(s["end_reason"], "failures")
        self.assertEqual(s["turns_attempted"], self.cfg.MAX_CONSECUTIVE_FAILURES)
        self.assertTrue(all(r["outcome"] == "failed_pull" for r in read(out / "turns.csv")))

    def test_motion_failure_is_a_failed_turn_with_a_message(self):
        s, out = self.run_trial(helpers.FailingMover())
        rows = read(out / "turns.csv")
        self.assertIn("no IK solution", rows[0]["mover_error"])
        self.assertEqual(rows[0]["outcome"], "failed_pull")

    def test_illegal_strategy_choice_is_rejected(self):
        strategies.REGISTRY["bad"] = "tests.helpers_bad"
        import sys, types
        m = types.ModuleType("tests.helpers_bad")
        m.select_block = lambda tower, candidates, cfg, rng, sensors=None: 1  # block 1 is a middle block
        sys.modules["tests.helpers_bad"] = m
        try:
            with self.assertRaises(ValueError):
                self.run_trial(FakeMover(), strategy="bad")
        finally:
            strategies.REGISTRY.pop("bad", None)
            sys.modules.pop("tests.helpers_bad", None)

    def test_trial_ends_when_nothing_is_left_to_pull(self):
        cfg = make_cfg(RECORD_REPLAY=False, RUNS_DIR=str(self.root / "r2"), FAKE_RUNS_DIR=str(self.root / "f2"), MAX_TURNS=50)
        s, _ = self.run_trial(FakeMover(), cfg=cfg)
        self.assertIn(s["end_reason"], ("no_candidates", "no_placement_slot"))


class TestResultsGuard(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.cfg = make_cfg(RECORD_REPLAY=False, RUNS_DIR=str(self.root / "runs"), FAKE_RUNS_DIR=str(self.root / "runs_fake"), MAX_TURNS=4)

    def tearDown(self):
        self.tmp.cleanup()

    def _run(self, mover, n=3):
        out = harness.run_directory(self.cfg, mover, "random", stamp="x")
        for i in range(n):
            harness.run_trial(self.cfg, i, mover, out, make_world=MockWorld, log=lambda *_: None)
        return out

    def test_fake_runs_are_refused(self):
        self._run(FakeMover())
        with self.assertRaises(results.FakeRunsError):
            results.read_trials(self.root / "runs_fake", self.cfg)

    def test_a_fake_row_copied_into_runs_is_refused(self):
        out = self._run(FakeMover())
        real_dir = self.root / "runs" / "random_copied"
        real_dir.mkdir(parents=True)
        (real_dir / "trials.csv").write_text((out / "trials.csv").read_text())
        with self.assertRaises(results.FakeRunsError):
            results.read_trials(self.root / "runs", self.cfg)

    def test_real_runs_build_a_table(self):
        self._run(helpers.RealLikeMover())
        rows = results.read_trials(self.root / "runs", self.cfg)
        table = results.build_table(rows)
        self.assertEqual(len(table), 1)
        self.assertEqual(table[0]["trials"], 3)
        self.assertIn("blocks", results.format_table(table))


if __name__ == "__main__":
    unittest.main()
