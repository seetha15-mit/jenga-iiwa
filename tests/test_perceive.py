"""Perception in games (stage 3): perceive.guesses, harness.perception_error, and the harness switch.
Pure Python: the cameras and estimate_block_poses are replaced, so no Drake or renderer is needed."""

import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from jenga import geometry as geo
from jenga import perceive
from jenga.evaluation import harness
from jenga.mover import FakeMover
from jenga.tower import TowerState
from tests.helpers import MockWorld, make_cfg


class TestGuesses(unittest.TestCase):
    def setUp(self):
        self.cfg = make_cfg()
        self.tower = TowerState(self.cfg)
        self.H = geo.block_dims(self.cfg)[2]

    def test_trial_start_uses_the_settled_height(self):
        g = perceive.guesses(self.tower, self.cfg, {})
        self.assertEqual(set(g), set(self.tower.slot_map.values()))
        for (layer, slot), b in self.tower.slot_map.items():
            self.assertAlmostEqual(g[b][2, 3], self.cfg.PLATFORM_HEIGHT_M + layer * self.H)  # no gaps
            np.testing.assert_allclose(g[b][:2, 3], self.tower.target_pose(layer, slot)[:2, 3])  # built x, y

    def test_a_block_in_the_same_slot_starts_from_its_last_perceived_pose(self):
        b = self.tower.slot_map[(2, 1)]
        X = geo.make_pose(0.001, 0.002, 0.3, yaw=0.1)
        g = perceive.guesses(self.tower, self.cfg, {b: ((2, 1), X)})
        np.testing.assert_allclose(g[b], X)

    def test_a_block_in_a_new_slot_starts_from_that_slot(self):
        b = self.tower.slot_map[(0, 0)]
        last = {b: ((0, 0), geo.make_pose(0.5, 0.5, 0.5))}
        top = self.tower.top_layer() + 1
        self.tower.remove_from_slot(b)
        self.tower.place(b, top, 0)
        g = perceive.guesses(self.tower, self.cfg, last)
        self.assertAlmostEqual(g[b][2, 3], self.cfg.PLATFORM_HEIGHT_M + top * self.H)
        np.testing.assert_allclose(g[b][:2, 3], self.tower.target_pose(top, 0)[:2, 3])


class TestPerceptionError(unittest.TestCase):
    def test_worst_errors_and_unseen_count(self):
        truth = {0: geo.make_pose(0, 0, 0.1), 1: geo.make_pose(0.1, 0, 0.1), 2: geo.make_pose(0.2, 0, 0.1)}
        seen = {b: X.copy() for b, X in truth.items()}
        seen[0][2, 3] += 0.0005   # 0.5 mm high
        seen[1][0, 3] += 0.002    # 2 mm sideways
        seen[2][2, 3] += 0.05     # way off, but nobody saw it: its guess, not scored
        cols = harness.perception_error(seen, truth, unseen=[2])
        self.assertEqual(cols, dict(perception_unseen=1, perception_max_height_mm=0.5, perception_max_pos_mm=2.0))


class TestHarnessSwitch(unittest.TestCase):
    def run_one(self, cfg):
        with tempfile.TemporaryDirectory() as tmp:
            harness.run_trial(cfg, 0, FakeMover(), Path(tmp), make_world=MockWorld, log=lambda *_: None)
            return (Path(tmp) / "turns.csv").read_text().splitlines()

    def test_off_reads_truth_and_leaves_the_columns_empty(self):
        with mock.patch.object(perceive, "observe") as observe:
            rows = self.run_one(make_cfg(RECORD_REPLAY=False, MAX_TURNS=2, PERCEPTION=False))
        observe.assert_not_called()
        header = rows[0].split(",")
        i = header.index("perception_unseen")
        self.assertEqual(rows[1].split(",")[i], "")

    def test_on_calls_observe_every_turn_and_logs_its_error(self):
        calls = []

        def fake_observe(world, tower, cfg, last):
            calls.append(dict(last))
            poses = {b: X.copy() for b, X in world.block_poses().items() if b in tower.slot_map.values()}
            for X in poses.values():
                X[2, 3] += 0.0003  # perception 0.3 mm high everywhere
            return poses, {b: ((0, 0), X) for b, X in poses.items()}, []

        with mock.patch.object(perceive, "observe", side_effect=fake_observe):
            rows = self.run_one(make_cfg(RECORD_REPLAY=False, MAX_TURNS=2, PERCEPTION=True))
        self.assertEqual(len(calls), 2)
        self.assertEqual(calls[0], {})      # a trial starts with nothing seen
        self.assertNotEqual(calls[1], {})   # the next turn starts from what was seen
        header = rows[0].split(",")
        row = dict(zip(header, rows[1].split(",")))
        self.assertEqual(row["perception_unseen"], "0")
        self.assertEqual(float(row["perception_max_height_mm"]), 0.3)


if __name__ == "__main__":
    unittest.main()
