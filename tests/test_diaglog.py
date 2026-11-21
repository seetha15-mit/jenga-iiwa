"""Development log (jenga/diaglog.py) and the forced first block.  Pure Python."""

import csv
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from jenga import config_check, diaglog
from jenga.evaluation import harness
from jenga.mover import TurnInfo
from jenga.tower import TowerState
from tests.helpers import MockWorld, RealLikeMover, make_cfg


class StandInWorld(MockWorld):
    """MockWorld plus what the 'hand' and 'joints' columns read.  The hand sits at the world origin."""

    def __init__(self, *a):
        super().__init__(*a)
        self.kin = SimpleNamespace(gripper_pose=lambda q: np.eye(4), joint_limit_margin=lambda q: np.radians(30))

    def arm_q(self):
        return np.zeros(7)

    def X_WR(self):
        return np.eye(4)


class TestStepLog(unittest.TestCase):
    def setUp(self):
        self.cfg = make_cfg(DIAG_LOG_LEVEL=2, DIAG_LOG_FEATURES=("block", "neighbours", "hand", "joints"))
        self.tower = TowerState(self.cfg)
        self.w = StandInWorld(self.cfg, self.tower, {}, False)

    def test_block_and_neighbour_columns(self):
        log = diaglog.StepLog(self.w, self.tower, 4, self.cfg)  # middle block of layer 1
        X = self.w.poses[4].copy()
        X[:3, 3] += X[:3, 0] * 0.03  # pushed 30 mm along its own long axis
        self.w.poses[4] = X
        self.w.poses[7][:3, 3] += [0, 0, 0.002]  # layer 2 (above) lifted 2 mm
        self.w.poses[3][:3, 3] += [0.001, 0, 0]  # same layer, 1 mm
        log.begin_step()
        log.step("push", SimpleNamespace(tracking_error_m=0.004))
        r = log.rows[0]
        self.assertAlmostEqual(r["block_along_mm"], 30.0)
        self.assertAlmostEqual(r["block_side_mm"], 0.0)
        self.assertEqual((r["worst_id"], r["worst_layer"], r["worst_mm"]), (7, 2, 2.0))
        self.assertEqual((r["above_mm"], r["same_layer_mm"], r["below_mm"]), (2.0, 1.0, 0.0))
        self.assertEqual((r["tracking_mm"], r["joint_margin_deg"]), (4.0, 30.0))
        self.assertEqual(set(r), set(log.columns))
        self.assertEqual([r[f"q{i}_deg"] for i in range(1, 8)], [0.0] * 7)  # StandInWorld's arm is at zero

    def test_moves_are_per_step_and_level_2_has_every_block(self):
        log = diaglog.StepLog(self.w, self.tower, 4, self.cfg)
        self.w.poses[7][:3, 3] += [0, 0, 0.002]
        log.step("push", None)
        log.step("roll_to_pull", None)  # nothing moved in this step
        self.assertEqual(log.rows[1]["worst_mm"], 0.0)
        self.assertEqual(log.rows[1]["worst_since_turn_mm"], 2.0)
        self.assertEqual(len(log.block_rows), 2 * len(self.tower.blocks))

    def test_features_choose_the_columns(self):
        cfg = make_cfg(DIAG_LOG_LEVEL=1, DIAG_LOG_FEATURES=("block",))
        log = diaglog.StepLog(self.w, self.tower, 4, cfg)
        log.step("push", None)
        self.assertEqual(list(log.rows[0]), ["step", "waypoint", "sim_t", "step_s", "error"] + diaglog._GROUPS["block"])
        self.assertEqual(log.block_rows, [])
        self.assertTrue(config_check.check_values(make_cfg(DIAG_LOG_FEATURES=("blok",))))


    def test_waypoint_rows(self):
        cfg = make_cfg(DIAG_LOG_LEVEL=1, DIAG_LOG_FEATURES=("neighbours", "hand", "waypoints"))
        log = diaglog.StepLog(self.w, self.tower, 4, cfg)
        self.assertTrue(log.per_waypoint)
        target = np.eye(4)
        target[:3, 3] = [0.001, 0.0, -0.002]  # the hand (at the origin) is 2 mm past the target along z, 1 mm short in x
        self.w.poses[1][:3, 3] += [0.003, 0, 0]  # layer 0 moves 3 mm before the first waypoint
        log.waypoint("push", 0, SimpleNamespace(X_RG=target, opening=0.11))
        self.w.poses[1][:3, 3] += [0.001, 0, 0]  # and 1 mm more before the step ends
        log.step("push", SimpleNamespace(tracking_error_m=0.0))
        wp, end = log.rows
        self.assertEqual((wp["waypoint"], wp["err_x_mm"], wp["err_z_mm"], wp["opening_cmd_mm"]), (0, -1.0, 2.0, 110.0))
        self.assertEqual((wp["below_mm"], end["below_mm"]), (3.0, 4.0))  # waypoint: since the last row; step: whole step
        self.assertEqual((end["waypoint"], end["err_x_mm"]), ("", ""))


class LoggingMover(RealLikeMover):
    """Teleports like RealLikeMover and leaves a development log of one step, like RealMover."""

    def do_turn(self, world, tower, block_id, target, cfg):
        self.last_diag = diaglog.StepLog(world, tower, block_id, cfg)
        world.teleport_block(block_id, tower.target_pose(*target))
        self.last_diag.step("teleport", None)
        return TurnInfo()


class TestHarnessDevOptions(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.cfg = make_cfg(RECORD_REPLAY=False, MAX_TURNS=1, EXTRACTION_METHOD="push", DIAG_LOG_LEVEL=2, DIAG_LOG_FEATURES=("block", "neighbours"))

    def tearDown(self):
        self.tmp.cleanup()

    def run_trial(self, cfg, mover):
        out = Path(self.tmp.name)
        return harness.run_trial(cfg, 0, mover, out, make_world=MockWorld, log=lambda *_: None), out

    def test_forced_first_block_and_step_files(self):
        self.cfg.DIAG_FIRST_BLOCK = 13
        s, out = self.run_trial(self.cfg, LoggingMover())
        self.assertEqual(s["first_block"], 13)
        with open(out / "steps.csv") as f:
            rows = list(csv.DictReader(f))
        self.assertEqual((rows[0]["block_id"], rows[0]["step"]), ("13", "teleport"))
        self.assertTrue((out / "step_blocks.csv").exists())

    def test_forced_block_must_be_legal(self):
        self.cfg.DIAG_FIRST_BLOCK = 16  # top layer: never a legal pull
        with self.assertRaises(ValueError):
            self.run_trial(self.cfg, LoggingMover())


if __name__ == "__main__":
    unittest.main()
